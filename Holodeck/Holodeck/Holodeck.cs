using System;
using System.Collections.Generic;
using System.Linq;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Audio;
using Microsoft.Xna.Framework.Content;
using Microsoft.Xna.Framework.GamerServices;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;
using Microsoft.Xna.Framework.Media;
using Henge3D.Physics;

namespace Henge3D.Holodeck
{
	public class Holodeck : Microsoft.Xna.Framework.Game
	{
		private IViewManager _viewManager;
		private IInputManager _inputManager;
		private IStateManager _stateManager;

		private PhysicsManager _physics;
		private SpriteFont _defaultFont;
		private SpriteBatch _spriteBatch;
		private List<Marker> _markers = new List<Marker>();

#if WINDOWS || WINDOWS_PHONE
		private RigidBody _pickedObject;
		private GrabConstraint _pickedForce;
		private float _pickedDistance;
#else
		int curScene;
#endif

		public Holodeck()
		{
			var gdm = new GraphicsDeviceManager(this);
			gdm.SynchronizeWithVerticalRetrace = false;
#if XBOX
			gdm.IsFullScreen = true;
			gdm.PreferredBackBufferHeight = 720;
			gdm.PreferredBackBufferWidth = 1280;
#elif WINDOWS_PHONE
            gdm.IsFullScreen = true;
            gdm.PreferredBackBufferHeight = 800;
            gdm.PreferredBackBufferWidth = 400;
#endif

			this.IsFixedTimeStep = false;
			TaskManager.IsThreadingEnabled = false;

			_viewManager = new ViewManager(this);
			_inputManager = new InputManager(this);
			_stateManager = new StateManager(this);
			_physics = new PhysicsManager(this);
			this.Components.Add(new PhysicsScene(this, _physics));

			Content.RootDirectory = "Content";
		}

		protected override void Initialize()
		{
			base.Initialize();

			_viewManager.SetProjection(0.1f, 100f, MathHelper.ToRadians(45f));
			_viewManager.Position = new Vector3(5f, 0f, 5f);
			_viewManager.UpAxis = Vector3.UnitZ;
			_viewManager.ForwardAxis = -Vector3.UnitX;
			_viewManager.MinPitch = MathHelper.ToRadians(-89.9f);
			_viewManager.MaxPitch = MathHelper.ToRadians(89.9f);

			var state = new FreeLookState(_stateManager);
			state.MovementSpeed = 10f;
			_stateManager.SetState(state);

			CreateScene(0);

#if WINDOWS
			Program.Proxy.SetModelList(
				new string[] {
					"models/small_cube",
					"models/sphere",
					"models/capsule",
					"models/obelisk",
					"models/triangle",
					"models/lblock"
				});
#endif
		}

		protected override void LoadContent()
		{
			base.LoadContent();

			_defaultFont = Content.Load<SpriteFont>("defaultFont");
			_spriteBatch = new SpriteBatch(this.GraphicsDevice);
		}

		protected override void Update(GameTime gameTime)
		{
			_inputManager.CaptureMouse = this.IsActive && _inputManager.MouseState.RightButton == ButtonState.Pressed;
#if WINDOWS
			_physics.Enabled = !Program.Proxy.IsPaused;
#endif

#if WINDOWS || WINDOWS_PHONE
			foreach (Keys k in _inputManager.KeysPressed)
			{
				switch (k)
				{
					case Keys.Space:
						SpawnSelectedObject();
						break;
					case Keys.R:
						SpawnRagdoll();
						break;
					case Keys.N:
						if (!_physics.Enabled)
						{
							_physics.Integrate((float)gameTime.ElapsedGameTime.TotalSeconds);
						}
						break;
					case Keys.D0:
					case Keys.D1:
					case Keys.D2:
					case Keys.D3:
					case Keys.D4:
					case Keys.D5:
					case Keys.D6:
					case Keys.D7:
					case Keys.D8:
					case Keys.D9:
						CreateScene(k - Keys.D0);
						break;
                    case Keys.Escape:
                    case Keys.Q:
                        this.Exit();
                        break;
				}
			}

			// object picking
			if (_inputManager.WasPressed(MouseButton.MiddleButton))
			{
				Segment s;
				s.P1 = GraphicsDevice.Viewport.Unproject(new Vector3(_inputManager.MouseState.X, _inputManager.MouseState.Y, 0f),
					_viewManager.Projection, _viewManager.View, Matrix.Identity);
				s.P2 = GraphicsDevice.Viewport.Unproject(new Vector3(_inputManager.MouseState.X, _inputManager.MouseState.Y, 1f),
					_viewManager.Projection, _viewManager.View, Matrix.Identity);
				float scalar;
				Vector3 point;
				var c = _physics.BroadPhase.Intersect(ref s, out scalar, out point);
				if (c != null && c is BodySkin)
				{
					_pickedObject = ((BodySkin)c).Owner;

					_pickedForce = new GrabConstraint(_pickedObject, point);
					_physics.Add(_pickedForce);
					_pickedDistance = scalar;
					_pickedObject.IsActive = true;
				}
			}
			else if (_inputManager.MouseState.MiddleButton == ButtonState.Pressed && _pickedObject != null)
			{
				Segment s;
				s.P1 = GraphicsDevice.Viewport.Unproject(new Vector3(_inputManager.MouseState.X, _inputManager.MouseState.Y, 0f),
					_viewManager.Projection, _viewManager.View, Matrix.Identity);
				s.P2 = GraphicsDevice.Viewport.Unproject(new Vector3(_inputManager.MouseState.X, _inputManager.MouseState.Y, 1f),
					_viewManager.Projection, _viewManager.View, Matrix.Identity);
				Vector3 diff, point;
				Vector3.Subtract(ref s.P2, ref s.P1, out diff);
				Vector3.Multiply(ref diff, _pickedDistance, out diff);
				Vector3.Add(ref s.P1, ref diff, out point);
				_pickedForce.WorldPoint = point;
				_pickedObject.IsActive = true;
			}
			else if (_pickedObject != null)
			{
				_physics.Remove(_pickedForce);
				_pickedObject = null;
			}
#else
			if (_inputManager.WasPressed(Buttons.RightShoulder)) CreateScene(++curScene);
			if (_inputManager.WasPressed(Buttons.LeftShoulder)) CreateScene(curScene == 0 ? 0 : --curScene);
			if (_inputManager.WasPressed(Buttons.A)) SpawnSelectedObject();
			if (_inputManager.WasPressed(Buttons.B)) SpawnRagdoll();
			if (_inputManager.WasPressed(Buttons.X)) 
				_physics.Enabled = !_physics.Enabled;
			if (_inputManager.WasPressed(Buttons.Y)) _physics.Integrate((float)gameTime.ElapsedGameTime.TotalSeconds);
			if (_inputManager.WasPressed(Buttons.Back)) this.Exit();
#endif
			UpdateFps(gameTime);

			base.Update(gameTime);
		}

		private void SpawnSelectedObject()
		{
			Vector3 vDir = _viewManager.Direction;

#if WINDOWS
			string asset = Program.Proxy.SelectedAsset;
			if (asset != null)
			{
				var thing = new SolidThing(this, this.Content.Load<Model>(asset));
				thing.SetWorld(
					_viewManager.Position + vDir * 2.0f,
					Quaternion.CreateFromAxisAngle(Vector3.UnitZ, _viewManager.Yaw) *
					Quaternion.CreateFromAxisAngle(Vector3.UnitY, _viewManager.Pitch));
				thing.LinearVelocity = vDir * Program.Proxy.SpawnVelocity;
				thing.AngularVelocity = Vector3.Transform(Program.Proxy.SpawnAngularVelocity, thing.Orientation);
				_physics.Add(thing);
			}
#else
			var rb = new SolidThing(this, this.Content.Load<Model>("models/small_cube"));
			rb.SetWorld(_viewManager.Position + vDir * 6f,
				Quaternion.CreateFromRotationMatrix(Matrix.CreateFromAxisAngle(Vector3.UnitY, _viewManager.Pitch) *
				Matrix.CreateFromAxisAngle(Vector3.UnitZ, _viewManager.Yaw)));
			rb.LinearVelocity = vDir * 20f;
			_physics.Add(rb);
#endif
		}

		private void SpawnRagdoll()
		{
			Vector3 vDir = _viewManager.Direction;

			var rd = new Ragdoll(this);
			rd.SetWorld(_viewManager.Position + vDir * 3f,
				Quaternion.CreateFromRotationMatrix(Matrix.CreateFromAxisAngle(Vector3.UnitY, _viewManager.Pitch) *
				Matrix.CreateFromAxisAngle(Vector3.UnitZ, _viewManager.Yaw)));
#if WINDOWS
			rd.SetLinearVelocity(vDir * Program.Proxy.SpawnVelocity);
#endif
			_physics.Add(rd);
			rd.Activate();
		}

		float deltaTime = 0f;
		int framesPerSecond;
		public void UpdateFps(GameTime gameTime)
		{
			float dt = (float)gameTime.ElapsedGameTime.TotalSeconds;
			float fps = 1 / dt;
			deltaTime += dt;

			if (deltaTime >= 1f)
			{
				framesPerSecond = (int)fps;
				deltaTime -= 1f;
#if WINDOWS
				Program.Proxy.SetFps(framesPerSecond);
#endif
			}
		}

		protected override void Draw(GameTime gameTime)
		{
			_viewManager.Device.RasterizerState = new RasterizerState()
			{
				CullMode = CullMode.CullClockwiseFace,
				FillMode = Constants.DebugCollisions ? FillMode.WireFrame : FillMode.Solid
			};
			_viewManager.Device.SamplerStates[0] = SamplerState.LinearWrap;

			base.Draw(gameTime);

#if WINDOWS
			if (Constants.DebugCollisions && _markers.Count > 0)
			{
				_viewManager.Device.RasterizerState = new RasterizerState()
				{
					CullMode = CullMode.None,
					FillMode = FillMode.Solid
				};
				for (int i = 0; i < _markers.Count; i++)
					_markers[i].Draw(_viewManager);
			}
#else
            _spriteBatch.Begin(SpriteSortMode.FrontToBack, BlendState.Additive);
            _spriteBatch.DrawString(_defaultFont, framesPerSecond.ToString(),
                new Vector2(GraphicsDevice.DisplayMode.TitleSafeArea.Left + 10f,
                    GraphicsDevice.DisplayMode.TitleSafeArea.Top + 10f), Color.AliceBlue);
            _spriteBatch.End();

            _viewManager.Device.BlendState = BlendState.Opaque;
            _viewManager.Device.DepthStencilState = DepthStencilState.Default;
#endif
        }

		static Random _rand = new Random();
		public void CreateScene(int sceneNumber)
		{
			_physics.Clear();
			_markers.Clear();

			Room room = new Room(this);
			_physics.Add(room);
			_physics.Gravity = new Vector3(0f, 0f, -9.8f);

			Model cubeModel = this.Content.Load<Model>("models/small_cube");
			Model obeliskModel = this.Content.Load<Model>("models/obelisk");
			Model sphereModel = this.Content.Load<Model>("models/sphere");
			Model capsuleModel = this.Content.Load<Model>("models/capsule");
			Model torusModel = this.Content.Load<Model>("models/torus");
			Model slabModel = this.Content.Load<Model>("models/slab");
			Model triangleModel = this.Content.Load<Model>("models/triangle");

			switch (sceneNumber)
			{
				case 1:
					{
						for (int i = 0; i < 12; i++)
						{
							var cube = new SolidThing(this, cubeModel);
							cube.SetWorld(new Vector3(0f, 0f, 0.25f + 0.51f * i));
							_physics.Add(cube);
						}
					}
					break;
				case 2:
					{
						for (int i = 0; i < 7; i++)
						{
							for (int j = 0; j < 7 - i; j++)
							{
								var cube = new SolidThing(this, cubeModel);
								cube.SetWorld(new Vector3(0f, 0.501f * j + 0.25f * i, 0.5f + 0.55f * i));
								_physics.Add(cube);
							}
						}
					}
					break;
				case 3:
					{
						for (int i = 0; i < 6; i++)
						{
							for (int j = 0; j < 6 - i; j++)
							{
								var cube = new SolidThing(this, cubeModel);
								cube.SetWorld(new Vector3(0f, 2.2f * j + 1f * i - 4.1f, 0.75f * i + 0.25f));
								_physics.Add(cube);
							}
						}
						for (int i = 0; i < 6; i++)
						{
							for (int j = 0; j < 5 - i; j++)
							{
								var plank = new SolidThing(this, obeliskModel);
								plank.SetWorld(new Vector3(0f, 2.2f * j + 1f * i + 1f - 4f, 0.75f * i + 0.65f),
									Quaternion.CreateFromAxisAngle(Vector3.UnitZ, MathHelper.ToRadians(90f)));
								_physics.Add(plank);
							}
						}
					}
					break;
				case 4:
					{
						int size = 9;
#if WINDOWS
#else
						size = 4;
#endif
						for (int i = 0; i < size; i++)
						{
							for (int j = 0; j < size - i; j++)
							{
								for (int k = 0; k < size - i; k++)
								{
									var sphere = new SolidThing(this, sphereModel);
									sphere.SetWorld(new Vector3(
										0.501f * j + 0.25f * i,
										0.501f * k + 0.25f * i,
										0.501f * i + 0.5f
										), Quaternion.Identity);
									_physics.Add(sphere);
								}
							}
						}
					}
					break;
				case 5:
					{

						var plank = new SolidThing(this, obeliskModel);
						plank.SetWorld(new Vector3(0.0f, 0.0f, 4.0f),
							Quaternion.CreateFromAxisAngle(Vector3.UnitY, MathHelper.ToRadians(15f)));
						MassProperties immovableMassProperties = new MassProperties(float.PositiveInfinity, Matrix.Identity);
						plank.MassProperties = immovableMassProperties;
						_physics.Add(plank);

						var sphere = new SolidThing(this, sphereModel);
						sphere.SetWorld(new Vector3(-4.9f, 0.0f, 9.0f), Quaternion.Identity);
						_physics.Add(sphere);


//                        int size = 9;
//#if WINDOWS
//#else
//                        size = 4;
//#endif
//                        var models = new Model[] { cubeModel, sphereModel };
//                        for (int i = 0; i < size; i++)
//                        {
//                            for (int j = 0; j < size - i; j++)
//                            {
//                                for (int k = 0; k < size - i; k++)
//                                {
//                                    var sphere = new SolidThing(this, i % 2 == 0 ? sphereModel : cubeModel);
//                                    sphere.SetWorld(new Vector3(
//                                        0.501f * j + 0.25f * i,
//                                        0.501f * k + 0.25f * i,
//                                        0.501f * i + 0.5f
//                                        ), Quaternion.Identity);
//                                    _physics.Add(sphere);
//                                }
//                            }
//                        }
					}
					break;
				case 6:
					{
						int size = 9;
#if WINDOWS
#else
						size = 4;
#endif
						var models = new Model[] { cubeModel, sphereModel, capsuleModel };
						for (int i = 0; i < size; i++)
						{
							for (int j = 0; j < size - i; j++)
							{
								for (int k = 0; k < size - i; k++)
								{
									var sphere = new SolidThing(this, models[_rand.Next(3)]);
									sphere.SetWorld(new Vector3(
										0.501f * j + 0.25f * i,
										0.501f * k + 0.25f * i,
										1f * i + 0.5f));
									_physics.Add(sphere);
								}
							}
						}
					}
					break;
				case 7:
					{
						var o = new SolidThing(this, torusModel);
						o.SetWorld(new Vector3(0f, 0f, 0.5f), Quaternion.CreateFromAxisAngle(Vector3.UnitY, -MathHelper.PiOver2));
						_physics.Add(o);

						o = new SolidThing(this, torusModel);
						o.SetWorld(new Vector3(0f, 0f, 4f), Quaternion.CreateFromAxisAngle(Vector3.UnitY, -MathHelper.PiOver4));
						_physics.Add(o);

						o = new SolidThing(this, slabModel);
						o.SetWorld(new Vector3(-4f, 4f, 2f));
						_physics.Add(o);

						o = new SolidThing(this, this.Content.Load<Model>("models/cone"));
						o.SetWorld(new Vector3(-4f, -4f, 1f), Quaternion.CreateFromAxisAngle(Vector3.UnitZ, MathHelper.PiOver2));
						_physics.Add(o);

						o = new SolidThing(this, cubeModel);
						o.SetWorld(new Vector3(-4f, 6.1f, 3f));
						_physics.Add(o);
					}
					break;
				case 8:
					{
						RigidBody oLast = null;
						for (int i = 0; i < 10; i++)
						{
							var o = new SolidThing(this, capsuleModel);
							o.SetWorld(new Vector3(0f, 0f, 9.5f - i));
							_physics.Add(o);
							if (i == 0)
							{
								var j = new PointConstraint(o, room, new Vector3(0f, 0f, 10f));
								j.IsCollisionEnabled = false;
								_physics.Add(j);
							}
							else
							{
								var j = new PointConstraint(oLast, o, new Vector3(0f, 0f, 10f - (float)i));
								j.IsCollisionEnabled = false;
								_physics.Add(j);
							}
							oLast = o;
						}

						var a = new SolidThing(this, cubeModel);
						a.SetWorld(new Vector3(1f, 0f, 0.25f));
						_physics.Add(a);
						var b = new SolidThing(this, cubeModel);
						b.SetWorld(new Vector3(1f, 0f, 0.75f));
						_physics.Add(b);
						var j2 = new RevoluteJoint(b, a, new Vector3(1.25f, 0f, 0.5f), Vector3.UnitY,
							0f, MathHelper.PiOver2);
						j2.IsCollisionEnabled = false;
						_physics.Add(j2);

						a = new SolidThing(this, cubeModel);
						a.SetWorld(new Vector3(1f, 1f, 0.25f));
						_physics.Add(a);
						b = new SolidThing(this, cubeModel);
						b.SetWorld(new Vector3(1f, 1f, 0.75f));
						_physics.Add(b);
						var j4 = new GenericConstraint(b, a, new Frame(new Vector3(1f, 1f, 0.5f)),
							Axes.All, Vector3.Zero, new Vector3(0f, 0f, 0.5f),
							Axes.All, Vector3.Zero, Vector3.Zero);
						j4.IsCollisionEnabled = false;
						_physics.Add(j4);

						a = new SolidThing(this, cubeModel);
						a.SetWorld(new Vector3(1f, 2f, 0.25f));
						_physics.Add(a);
						b = new SolidThing(this, cubeModel);
						b.SetWorld(new Vector3(1f, 2f, 0.75f));
						_physics.Add(b);
						var j5 = new GenericConstraint(b, a, new Frame(new Vector3(1f, 2f, 0.5f)),
							Axes.All, new Vector3(-0.125f, -0.125f, 0f), new Vector3(0.125f, 0.125f, 0f),
							Axes.All, Vector3.Zero, Vector3.Zero);
						j5.IsCollisionEnabled = false;
						_physics.Add(j5);

						a = new SolidThing(this, sphereModel);
						a.SetWorld(new Vector3(2f, 0f, 2f));
						_physics.Add(a);
						b = new SolidThing(this, sphereModel);
						b.SetWorld(new Vector3(2f, 0f, 1f));
						_physics.Add(b);
						var g1 = new SpringForce(a, b, Vector3.Zero, Vector3.Zero, 1f, 5f, 0.05f);
						_physics.Add(g1);
						var j3 = new WorldPointConstraint(a, new Vector3(2f, 0f, 2f));
						_physics.Add(j3);
					}
					break;
				case 9:
					{
						var a = new SolidThing(this, sphereModel);
						a.Skin.Remove(a.Skin[0]);
						a.Skin.Add(new SpherePart(new Sphere(Vector3.Zero, 0.25f)), new Material(0f, 0.5f));
						a.SetWorld(7.0f, new Vector3(0f, 0f, 5f), Quaternion.Identity);
						a.MassProperties = MassProperties.Immovable;
						_physics.Add(a);

						_physics.Add(new SingularityForce(new Vector3(0f, 0f, 5f), 1E12f));

						_physics.Gravity = Vector3.Zero;

						var b = new SolidThing(this, cubeModel);
						b.SetWorld(new Vector3(0f, 0f, 8f),
							Quaternion.CreateFromAxisAngle(Vector3.UnitX, MathHelper.PiOver4 / 2.0f) *
							Quaternion.CreateFromAxisAngle(Vector3.UnitY, MathHelper.PiOver4)
							);
						_physics.Add(b);
					}
					break;
				default:
					break;
			}
		}
	}
}
