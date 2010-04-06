using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace Henge3D.Holodeck
{
	public interface IStateManager
	{
		GameState Current { get; }
		Game Game { get; }
		void SetState(GameState state);
	}
}
