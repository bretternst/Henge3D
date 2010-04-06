using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace Henge3D.Holodeck
{
	public abstract class GameState
	{
		private IStateManager _manager;

		public GameState(IStateManager stateManager)
		{
			_manager = stateManager;
		}

		public virtual void Start()
		{
		}

		public virtual void End()
		{
		}

		public virtual void Update(GameTime time)
		{
		}

		public IStateManager Manager { get { return _manager; } }
	}
}
