using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Henge3D.Physics;

namespace Henge3D.Holodeck
{
	class ScaledCapsule : RigidBody, IVisible
	{
		private Model _model;
		private Vector3 _diffuseColor;
		private Matrix _meshTransform;

		public ScaledCapsule(Game game, float length, float radius, Vector3 color)
		{
			_model = game.Content.Load<Model>("models/capsule");
			_meshTransform = _model.Meshes[0].ParentBone.Transform *
				Matrix.CreateScale(new Vector3(radius / 0.25f, radius / 0.25f, length + (radius * 2)));
			_diffuseColor = color;

			Vector3 dummy, p1 = new Vector3(0f, 0f, length / 2f), p2 = new Vector3(0f, 0f, -length / 2f);
			this.MassProperties = MassProperties.FromCapsule(1f, p1, p2, radius, out dummy);

			this.Skin.Add(new CapsulePart(new Capsule(p1, p2, radius)), new Material(0f, 0.5f));

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
