using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace Henge3D.Holodeck
{
	public sealed class StateManager : GameComponent, IStateManager
	{
		static private Game _currentGame;

		public static Game CurrentGame { get { return _currentGame; } }


		private GameState _current = null;

		public StateManager(Game game)
			: base(game)
		{
			_currentGame = game;
			this.Game.Services.AddService(typeof(IStateManager), this);
			this.Game.Components.Add(this);
		}

		public GameState Current { get { return _current; } }

		public override void Update(GameTime gameTime)
		{
			if (_current != null)
			{
				_current.Update(gameTime);
			}
			base.Update(gameTime);
		}

		public void SetState(GameState state)
		{
			if (_current != null)
			{
				_current.End();
			}
			_current = state;
			if (_current != null)
			{
				_current.Start();
			}
		}
	}
}
