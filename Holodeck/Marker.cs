using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;

namespace Henge3D.Holodeck
{
	public class Marker : IVisible
	{
		private static BasicEffect _effect;
		private static VertexDeclaration _vDec;
		private Vector3 _position;
		private Color _color;
		private float _rotation;

		public Marker(Game game, Vector3 position, Color color, float rotation)
		{
			this._position = position;
			this._color = color;
			this._rotation = rotation;
			if (_effect == null)
			{
				_effect = new BasicEffect(game.GraphicsDevice, null);
				_effect.AmbientLightColor = Vector3.One;
				_effect.VertexColorEnabled = true;
				_vDec = new VertexDeclaration(game.GraphicsDevice, VertexPositionColor.VertexElements);
			}
		}

		public void Draw(IViewManager view)
		{
			view.Device.VertexDeclaration = _vDec;
			view.Device.RenderState.CullMode = CullMode.None;
			view.Device.RenderState.FillMode = FillMode.Solid;
			_effect.Projection = view.Projection;
			_effect.View = view.View;
			_effect.World = Matrix.CreateScale(0.2f) * 
				Matrix.CreateRotationX(_rotation) *
				Matrix.CreateTranslation(_position);
			_effect.Begin();
			foreach (EffectPass pass in _effect.CurrentTechnique.Passes)
			{
				pass.Begin();
				view.Device.DrawUserPrimitives(PrimitiveType.TriangleList,
					new VertexPositionColor[] {
					new VertexPositionColor(new Vector3(0.0f, -1.0f, 0.0f), _color),
					new VertexPositionColor(new Vector3(0.0f, 1.0f, 0.0f), _color),
					new VertexPositionColor(new Vector3(0.0f, 0.0f, 1.0f), _color),
				}, 0, 1);
				pass.End();
			}
			_effect.End();
		}
	}
}
