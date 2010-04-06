using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Input;

namespace Henge3D.Holodeck
{
	public enum MouseButton
	{
		LeftButton,
		MiddleButton,
		RightButton,
		XButton1,
		XButton2
	}

	public interface IInputManager
	{
		GamePadState GamePadState { get; }
		MouseState MouseState { get; }
		KeyboardState KeyboardState { get; }
		Vector2 MouseDelta { get; }
		float MouseSensitivity { get; set; }
		bool CaptureMouse { get; set; }
		bool WasPressed(Buttons button);
		bool WasPressed(MouseButton button);
		IEnumerable<Keys> KeysPressed { get; }
	}
}
