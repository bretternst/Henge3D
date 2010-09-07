using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;
using Henge3D.Pipeline;

namespace Henge3D.Physics
{
	/// <summary>
	/// Provides a signature for event handlers that respond to collision events.
	/// </summary>
	/// <param name="sender">The rigid body object firing the event.</param>
	/// <param name="other">The rigid body object that the sender collided with.</param>
	/// <returns></returns>
	public delegate bool CollisionEventHandler(RigidBody sender, RigidBody other);

	/// <summary>
	/// Provides a signature for event handlers that respond to separation events.
	/// </summary>
	/// <param name="sender">The rigid body object firing the event.</param>
	/// <param name="other">The rigid body object that the sender separated from.</param>
	public delegate void SeparationEventHandler(RigidBody sender, RigidBody other);

	[Flags]
	internal enum ContactStateFlags
	{
		IsInContact = 1,
		IsSuppressed = 2,
		WasInContact = 4
	}

	/// <summary>
	/// Represents a single rigid body object, defined as a form composed of one or more collision skin parts. A rigid body always acts
	/// as a single unit which can be moved around the world and interact with other bodies through contact or other constraints.
	/// </summary>
	public class RigidBody
	{
		private PhysicsManager _manager;
		private BodySkin _skin;
		private Island _island;
		private float _inactiveTime = 0f;
		private bool _isActive = true, _isMovable = true, _isFast = false, _isWeightless = false;
		private List<Constraint> _contacts;
		private List<Constraint> _constraints;
		private Dictionary<RigidBody, int> _contactStates;

		internal Transform World, WorldInverse;
		internal Vector3 Force, Torque;
		internal TransformDelta Velocity;
		internal MassProperties Mass, MassWorld;

		/// <summary>
		/// Gets a reference to the physics implementation managing the body.
		/// </summary>
		public PhysicsManager Manager { get { return _manager; } internal set { _manager = value; } }

		/// <summary>
		/// Gets a reference to the body's collision skin.
		/// </summary>
		public BodySkin Skin { get { return _skin; } }

		/// <summary>
		/// Gets the current position of the body in world coordinates.
		/// </summary>
		public Vector3 Position { get { return World.Position; } }

		/// <summary>
		/// Gets the current orientation of the body in world-space.
		/// </summary>
		public Quaternion Orientation { get { return World.Orientation; } }

		/// <summary>
		/// Gets or sets the current linear velocity of the body, defined as a direction and magnitude.
		/// </summary>
		public Vector3 LinearVelocity { get { return Velocity.Linear; } set { Velocity.Linear = value; } }

		/// <summary>
		/// Gets or sets the current angular velocity of the body, defined as an axis direction and magnitude.
		/// </summary>
		public Vector3 AngularVelocity { get { return Velocity.Angular; } set { Velocity.Angular = value; } }

		/// <summary>
		/// Gets or sets the mass properties of the body.
		/// </summary>
		public MassProperties MassProperties { get { return Mass; } set { Mass = value; _isMovable = Mass.Mass < float.PositiveInfinity && Mass.MassInverse < float.PositiveInfinity; } }

		/// <summary>
		/// Gets a value indincating whether the object is movable. An body with infinite mass is considered immobile.
		/// </summary>
		public bool IsMovable { get { return _isMovable; } }

		/// <summary>
		/// Gets or sets a value indicating whether the body should be affected by gravitational forces.
		/// </summary>
		public bool IsWeightless { get { return _isWeightless; } set { _isWeightless = value; } }

		/// <summary>
		/// Gets the list of all constraints currently being applied to the body. Constraints in this list may be shared with other bodies.
		/// </summary>
		public IList<Constraint> Constraints { get { return _constraints; } }

		/// <summary>
		/// The current transform applied to the rigid body, indicating its position and orientation. This can be applied to a related visual
		/// entity to keep it in sync.
		/// </summary>
		public Transform Transform;

		/// <summary>
		/// Fires when this rigid body first comes into contact with another rigid body. The handler is only called during the first frame
		/// in which the bodies are in contact. If the handler returns true, the collision will be suppressed and the bodies will
		/// inter-penetrate until they are separated.
		/// </summary>
		public CollisionEventHandler OnCollision;

		/// <summary>
		/// Fires when this rigid body is no longer in contact with another rigid body.
		/// </summary>
		public SeparationEventHandler OnSeparation;

		internal Island Island { get { return _island; } set { _island = value; } }
		internal IList<Constraint> Contacts { get { return _contacts; } }
		internal bool CanDeactivate { get { return _inactiveTime >= _manager.DeactivationTime; } }
		internal bool IsFast { get { return _isFast; } }

		internal Dictionary<RigidBody, ContactStateFlags> ContactStates;

		/// <summary>
		/// Construct a new rigid body.
		/// </summary>
		public RigidBody()
		{
			Mass = MassProperties.Immovable;
			Transform = World = Transform.Identity;
			_isMovable = false;
			_contacts = new List<Constraint>();
			_constraints = new List<Constraint>();
			_skin = new BodySkin();
			_skin.Owner = this;
		}

		/// <summary>
		/// Construct a new rigid body.
		/// </summary>
		/// <param name="skin">A pre-constructed body skin to apply.</param>
		public RigidBody(BodySkin skin)
			: this()
		{
			_skin = skin;
			_skin.Owner = this;
		}

		/// <summary>
		/// Construct a new rigid body using the pre-compiled physics model.
		/// </summary>
		/// <param name="model"></param>
		public RigidBody(RigidBodyModel model)
			: this()
		{
			this.InitializeFromModel(model);
		}

		public RigidBody(RigidBodyModel model, BodySkin skin)
			: this()
		{
			_skin = skin;
			_skin.Owner = this;
			this.InitializeFromModel(model);
		}

		/// <summary>
		/// Gets or sets a value indicating whether the body is currently active. A body may be deactivated by the physics implementation
		/// if its movement has not exceeded a certain threshold for a period of time. Conversely, bodies may be re-activated automatically
		/// when certain events occur, such as a collision with an active body.
		/// </summary>
		public bool IsActive
		{
			get { return _isActive && _isMovable; }
			set
			{
				if (value != _isActive)
					_inactiveTime = value ? 0f : float.MaxValue;
				_isActive = value;
			}
		}

		/// <summary>
		/// Set the position of the body.
		/// </summary>
		/// <param name="position">The new position in world coordinates.</param>
		public void SetWorld(Vector3 position)
		{
			World = new Transform(World.Scale, ref position, ref World.Orientation);
			_skin.ApplyTransform(ref World);
			UpdateWorld();
		}

		/// <summary>
		/// Set the position and orientation of the body.
		/// </summary>
		/// <param name="position">The new position in world coordinates.</param>
		/// <param name="orientation">The new orientation in world-space.</param>
		public void SetWorld(Vector3 position, Quaternion orientation)
		{
			World = new Transform(World.Scale, ref position, ref orientation);
			_skin.ApplyTransform(ref World);
			UpdateWorld();
		}

		/// <summary>
		/// Set the scale, position and orientation of the body.
		/// </summary>
		/// <param name="scale">The new scaling factor.</param>
		/// <param name="position">The new position in world coordinates.</param>
		/// <param name="orientation">The new orientation in world-space.</param>
		public void SetWorld(float scale, Vector3 position, Quaternion orientation)
		{
			World = new Transform(scale, ref position, ref orientation);
			_skin.ApplyTransform(ref World);
			UpdateWorld();
		}

		/// <summary>
		/// Set the current linear and angular velocity of the body.
		/// </summary>
		/// <param name="linear">The new linear velocity.</param>
		/// <param name="angular">The new angular velocity, defined as an axis direction and magnitude.</param>
		public void SetVelocity(Vector3 linear, Vector3 angular)
		{
			Velocity = new TransformDelta(ref linear, ref angular);
		}

		/// <summary>
		/// Apply an impulse to the body, changing its velocity by applying an instantaneous pseudo-force to a specific position on the body.
		/// </summary>
		/// <param name="impulse">The direction and magnitude of impulse to apply.</param>
		/// <param name="offset">The position in world coordinates at which to apply the impulse.</param>
		public void ApplyImpulse(ref Vector3 impulse, ref Vector3 offset)
		{
			if (!IsActive) return;

			float massInv = MassProperties.MassInverse;

			Vector3 v;
			Vector3.Multiply(ref impulse, massInv, out v);

			Vector3 w;
			Vector3.Cross(ref offset, ref impulse, out w);
			Vector3.Transform(ref w, ref MassWorld.InertiaInverse, out w);

			Velocity.Add(ref v, ref w);
		}

		/// <summary>
		/// Apply an angular impulse to the body, changing its angular velocity by applying an instantaneous pseudo-torque.
		/// </summary>
		/// <param name="impulse">The axis direction and magnitude of impulse to apply.</param>
		public void ApplyAngularImpulse(ref Vector3 impulse)
		{
			Vector3 w, zero = Vector3.Zero;
			Vector3.Transform(ref impulse, ref MassWorld.InertiaInverse, out w);
			Velocity.Add(ref zero, ref w);
		}

		/// <summary>
		/// Apply a linear impulse for the current frame only, affecting the body's position by applying an instantaneous pseudo-force
		/// at a specific point on the body.
		/// </summary>
		/// <param name="impulse">The direction and magnitude of the impulse to apply.</param>
		/// <param name="offset">The point in world coordinates at which to apply the impulse.</param>
		public void ApplyFlashImpulse(ref Vector3 impulse, ref Vector3 offset)
		{
			if (!IsActive) return;

			float massInv = MassProperties.MassInverse;

			Vector3 v;
			Vector3.Multiply(ref impulse, massInv, out v);

			Vector3 w;
			Vector3.Cross(ref offset, ref impulse, out w);
			Vector3.Transform(ref w, ref MassWorld.InertiaInverse, out w);
			Vector3.Multiply(ref w, 0.25f, out w);

			var delta = new TransformDelta(ref v, ref w);
			World.ApplyDelta(1f, ref delta);
			UpdateWorld();
		}

		/// <summary>
		/// Apply an angular impulse for the current frame only, affecting the body's position by applying an instantaneous pseudo-torque.
		/// </summary>
		/// <param name="impulse">The axis direction and magnitude of impulse to apply.</param>
		public void ApplyAngularFlashImpulse(ref Vector3 impulse)
		{
			Vector3 w, zero = Vector3.Zero;
			Vector3.Transform(ref impulse, ref MassWorld.InertiaInverse, out w);
			var delta = new TransformDelta(ref zero, ref w);
			World.ApplyDelta(1f, ref delta);
			UpdateWorld();
		}

		/// <summary>
		/// Apply a force to the body over the current frame. The force is applied at the body's center of mass.
		/// </summary>
		/// <param name="force">The direction and magnitude of force to apply.</param>
		public void ApplyForce(ref Vector3 force)
		{
			if (this.IsActive)
			{
				Vector3.Add(ref Force, ref force, out Force);
			}
		}

		/// <summary>
		/// Apply a force to the body over the current frame. The force is applied at the specified position.
		/// </summary>
		/// <param name="force">The direction and magnitude of force to apply.</param>
		/// <param name="offset">The offset from the center of mass in world-space at which to apply force.</param>
		public void ApplyForce(ref Vector3 force, ref Vector3 offset)
		{
			if (this.IsActive)
			{
				Vector3.Add(ref Force, ref force, out Force);

				Vector3 t;
				Vector3.Cross(ref offset, ref force, out t);
				Vector3.Add(ref Torque, ref t, out Torque);
			}
		}

		/// <summary>
		/// Apply torque to the body.
		/// </summary>
		/// <param name="torque">The axis direction and magnitude of torque to apply.</param>
		public void ApplyTorque(ref Vector3 torque)
		{
			if (this.IsActive)
			{
				Vector3.Add(ref Torque, ref torque, out Torque);
			}
		}

		/// <summary>
		/// Gets the velocity of a specific point on an object by considering both linear and angular velocity.
		/// </summary>
		/// <param name="delta">The offset from the center of mass in world-space.</param>
		/// <param name="v">Returns the linear velocity of the specified point.</param>
		public void GetVelocityAtPoint(ref Vector3 delta, out Vector3 v)
		{
			Vector3.Cross(ref Velocity.Angular, ref delta, out v);
			Vector3.Add(ref Velocity.Linear, ref v, out v);
		}

		/// <summary>
		/// Freeze the body so that it cannot be moved by any force.
		/// </summary>
		public void Freeze()
		{
			_isMovable = false;
		}

		/// <summary>
		/// Unfreeze the body so that forces can act upon it.
		/// </summary>
		public void Unfreeze()
		{
			_isMovable = Mass.Mass < float.PositiveInfinity;
		}

		internal void IntegrateForce(float dt)
		{
			if (this.IsActive)
			{
				Vector3 v;
				Vector3.Multiply(ref Force, dt * MassProperties.MassInverse, out v);

				Vector3 w;
				Vector3.Multiply(ref Torque, dt, out w);
				Vector3.Transform(ref w, ref MassWorld.InertiaInverse, out w);

				Velocity.Add(ref v, ref w);

				// adjust the body's collision skin
				// - for fast moving objects: swept tests are used, so leave the position as-is, but sweep the skin's bounding box
				// - for slow moving objects: the estimated new position is used
				Vector3 delta;
				Vector3.Multiply(ref this.Velocity.Linear, dt, out delta);
				_isFast = delta.LengthSquared() >= this.Manager.SweepThresholdSquared;

				Transform world;
				if (_isFast)
				{
					world = this.World;
					_skin.ApplyTransform(ref world);
					_skin.ApplySweep(ref delta);
				}
				else
				{
					IntegrateVelocity(dt, out world);
					_skin.ApplyTransform(ref world);
				}
			}
		}

		internal void IntegrateVelocity(float dt)
		{
			if (IsActive)
			{
				IntegrateVelocity(dt, out World);
				DampVelocity();

				if (Velocity.Linear.LengthSquared() > _manager.ActiveLinearThresholdSquared ||
					Velocity.Angular.LengthSquared() > _manager.ActiveAngularThresholdSquared)
				{
					_inactiveTime = 0f;
				}
				else
				{
					_inactiveTime += dt;
				}
				UpdateWorld();
			}
			Force = Vector3.Zero;
			Torque = Vector3.Zero;
		}

		private void IntegrateVelocity(float dt, out Transform world)
		{
			world = World;
			Velocity.Clamp(_manager.LinearLimit, _manager.AngularLimit);
			world.ApplyDelta(dt, ref Velocity);
		}

		private void UpdateWorld()
		{
			World.Invert(out WorldInverse);
			MassProperties.Transform(ref Mass, ref World, out MassWorld);
			Transform = World;
		}

		private void DampVelocity()
		{
			float f = 1f - (1f - Manager.LinearDamping) * Manager.TimeStep;
			Vector3.Multiply(ref Velocity.Linear, f, out Velocity.Linear);
			f = 1f - (1f - Manager.AngularDamping) * Manager.TimeStep;
			Vector3.Multiply(ref Velocity.Angular, f, out Velocity.Angular);
		}

		private void InitializeFromModel(RigidBodyModel model)
		{
			MassProperties = model.MassProperties;
			for (int i = 0; i < model.Parts.Length; i++)
			{
				_skin.Add(model.Parts[i].ToCompositionPart(), model.Materials[i]);
			}
		}
	}
}
