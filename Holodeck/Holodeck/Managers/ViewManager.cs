using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;

namespace Henge3D.Holodeck
{
	public sealed class ViewManager : DrawableGameComponent, IViewManager
	{
		const float DefaultMaxPitch = float.PositiveInfinity;
		const float DefaultMinPitch = float.NegativeInfinity;
		static readonly Vector3 DefaultUpAxis = Vector3.UnitY;
		static readonly Vector3 DefaultForwardAxis = -Vector3.UnitZ;
		static readonly Vector3 DefaultSideAxis = Vector3.UnitX;

		private GraphicsDevice _device;
		private Matrix _viewMatrix;
		private Matrix _projectionMatrix;
		private Vector3 _position;
		private Vector3 _upAxis = DefaultUpAxis;
		private Vector3 _forwardAxis = DefaultForwardAxis;
		private Vector3 _sideAxis = DefaultSideAxis;
		private float _pitch = 0f;
		private float _yaw = 0f;
		private float _maxPitch = DefaultMaxPitch;
		private float _minPitch = DefaultMinPitch;

		public ViewManager(Game game)
			: base(game)
		{
			this.Game.Services.AddService(typeof(IViewManager), this);
			this.Game.Components.Add(this);
		}

		public GraphicsDevice Device { get { return this.Game.GraphicsDevice; } }

		public Vector3 UpAxis
		{
			get { return _upAxis; }
			set
			{
				_upAxis = value;
				Vector3.Cross(ref _forwardAxis, ref _upAxis, out _sideAxis);
			}
		}

		public Vector3 ForwardAxis
		{
			get { return _forwardAxis; }
			set
			{
				_forwardAxis = value;
				Vector3.Cross(ref _forwardAxis, ref _upAxis, out _sideAxis);
			}
		}

		public Vector3 Position { get { return _position; } set { _position = value; } }

		public Vector3 Direction
		{
			get
			{
				return Vector3.TransformNormal(
					_forwardAxis,
					Matrix.CreateFromAxisAngle(_sideAxis, _pitch) * Matrix.CreateFromAxisAngle(_upAxis, _yaw));
			}
			set
			{
				this.Pitch = (float)Math.Asin(Vector3.Dot(_upAxis, value));
				this.Yaw = (float)Math.Atan2(
					Vector3.Dot(-_forwardAxis, value), Vector3.Dot(-_sideAxis, value));
			}
		}

		public Matrix Projection { get { return _projectionMatrix; } set { _projectionMatrix = value; } }

		public Matrix View { get { return _viewMatrix; } }

		public float MaxPitch { get { return _maxPitch; } set { _maxPitch = value; } }

		public float MinPitch { get { return _minPitch; } set { _minPitch = value; } }

		public float Pitch
		{
			get { return _pitch; }
			set
			{
				_pitch = MathHelper.Clamp(value, _minPitch, _maxPitch);
			}
		}

		public float Yaw
		{
			get { return _yaw; }
			set
			{
				_yaw = (value >= MathHelper.TwoPi) ? value % MathHelper.TwoPi : value;
			}
		}

		public void SetProjection(float viewPlaneNear, float viewPlaneFar, float fieldOfView)
		{
			Matrix.CreatePerspectiveFieldOfView(
				fieldOfView,
				this.Game.GraphicsDevice.Viewport.AspectRatio,
				viewPlaneNear,
				viewPlaneFar,
				out _projectionMatrix);
		}

		public void SetProjection(float viewDistance)
		{
			Matrix.CreateOrthographicOffCenter(
				0f,
				this.Game.GraphicsDevice.Viewport.Width,
				0f,
				this.Game.GraphicsDevice.Viewport.Height,
				0f,
				viewDistance,
				out _projectionMatrix);
		}

		public void Move(Vector3 delta)
		{
			if (delta != Vector3.Zero)
			{
				Vector3 sideAxis;
				Vector3.Cross(ref _forwardAxis, ref _upAxis, out sideAxis);
				_position += Vector3.Transform(
					delta,
					Matrix.CreateFromAxisAngle(sideAxis, _pitch) * Matrix.CreateFromAxisAngle(_upAxis, _yaw));
			}
		}

		public override void Initialize()
		{
			base.Initialize();

			_device = this.Game.GraphicsDevice;
			_position = Vector3.Zero;
			_projectionMatrix = Matrix.Identity;
		}

		public override void Draw(GameTime gameTime)
		{
			_device.Clear(Color.Black);

			Vector3 look = this.Direction;
			Vector3.Add(ref _position, ref look, out look);

			Matrix.CreateLookAt(
				ref _position,
				ref look,
				ref _upAxis,
				out _viewMatrix);

			base.Draw(gameTime);
		}
	}
}
