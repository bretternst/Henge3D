using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Henge3D.Physics;

namespace Henge3D.Holodeck
{
	class ScaledSphere : RigidBody, IVisible
	{
		private Model _model;
		private Vector3 _diffuseColor;
		private Matrix _meshTransform;

		public ScaledSphere(Game game, float radius, Vector3 color)
		{
			_model = game.Content.Load<Model>("models/sphere");
			_diffuseColor = color;

			_meshTransform = Matrix.CreateScale(radius / 0.25f);

			this.MassProperties = MassProperties.FromSphere(1f, Vector3.Zero, radius);

			this.Skin.Add(new SpherePart(new Sphere(Vector3.Zero, radius)), new Material(0f, 0.5f));

			foreach (var mesh in _model.Meshes)
			{
				foreach (BasicEffect effect in mesh.Effects)
				{
					effect.EnableDefaultLighting();
					effect.AmbientLightColor = Vector3.One * 0.75f;
					effect.SpecularColor = Vector3.One;
					effect.PreferPerPixelLighting = true;
					effect.CommitChanges();
				}
			}
		}

		public void Draw(IViewManager view)
		{
			view.Device.RenderState.CullMode = CullMode.CullCounterClockwiseFace;

			foreach (var mesh in _model.Meshes)
			{
				foreach (BasicEffect effect in mesh.Effects)
				{
					effect.World = _meshTransform * Transform.Combined;
					effect.View = view.View;
					effect.Projection = view.Projection;
					effect.DiffuseColor = _diffuseColor;
					if (!this.IsActive)
					{
						effect.DiffuseColor *= 0.5f;
					}
				}
				mesh.Draw();
			}
		}
	}
}
