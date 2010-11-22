using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Input;

namespace Henge3D.Holodeck
{
	public class FreeLookState : GameState
	{
		private IViewManager _camera;
		private IInputManager _input;

		float movementSpeed = 1f;

		public FreeLookState(IStateManager stateManager) :
			base(stateManager)
		{
			_camera = (IViewManager)stateManager.Game.Services.GetService(typeof(IViewManager));
			_input = (IInputManager)stateManager.Game.Services.GetService(typeof(IInputManager));

			_input.CaptureMouse = true;
		}

		public override void Update(GameTime time)
		{
			float delta = (float)time.ElapsedGameTime.TotalSeconds;
			Vector3 moveVector = Vector3.Zero;
#if WINDOWS
			_camera.Pitch += _input.MouseDelta.Y * _input.MouseSensitivity;
			_camera.Yaw -= _input.MouseDelta.X * _input.MouseSensitivity;

			if (_input.KeyboardState.IsKeyDown(Keys.E) || _input.KeyboardState.IsKeyDown(Keys.W))
			{
				moveVector.X -= 1f;
			}
			if (_input.KeyboardState.IsKeyDown(Keys.A))
			{
				moveVector.Y -= 1f;
			}
			if (_input.KeyboardState.IsKeyDown(Keys.D))
			{
				moveVector.Y += 1f;
			}
			if (_input.KeyboardState.IsKeyDown(Keys.S))
			{
				moveVector.X += 1f;
			}
#else
			_camera.Pitch -= _input.GamePadState.ThumbSticks.Right.Y * delta * 0.8f;
			_camera.Yaw -= _input.GamePadState.ThumbSticks.Right.X * delta;

			moveVector.X -= _input.GamePadState.ThumbSticks.Left.Y;
			moveVector.Y += _input.GamePadState.ThumbSticks.Left.X;
#endif

			if (moveVector != Vector3.Zero)
			{
				moveVector.Normalize();
				moveVector *= movementSpeed * delta;
				_camera.Move(moveVector);
			}
		}

		public float MovementSpeed { get { return movementSpeed; } set { movementSpeed = value; } }
	}
}
