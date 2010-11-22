using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;
using Henge3D.Physics;

namespace Henge3D.Holodeck
{
	public interface IVisible
	{
		void Draw(IViewManager view);
	}

	public class PhysicsScene : DrawableGameComponent
	{
		private PhysicsManager _physics;
		private IViewManager _view;

		public PhysicsScene(Game game, PhysicsManager physics)
			: base(game)
		{
			_physics = physics;
		}

		public override void Initialize()
		{
			base.Initialize();

			_view = this.Game.Services.GetService(typeof(IViewManager)) as IViewManager;
			if (_view == null)
				throw new Exception("No view manager is registered.");
		}

		public override void Draw(GameTime gameTime)
		{
			for (int i = 0; i < _physics.Bodies.Count; i++)
			{
				var iv = _physics.Bodies[i] as IVisible;
				if (iv != null)
					iv.Draw(_view);
			}
		}
	}
}
