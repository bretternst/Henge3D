using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Henge3D.Physics;
using Henge3D.Pipeline;

namespace Henge3D.Holodeck
{
	public class SolidThing : RigidBody, IVisible
	{
		private static Random _colorRand = new Random();

		private Model _model;
		private Matrix[] _meshTransforms;
		private bool _isColorRandom;
		private Vector3 _diffuseColor;

		public SolidThing(Game game, Model model)
			: this(game, model, true)
		{
		}

		public SolidThing(Game game, Model model, bool isColorRandom)
			: base((RigidBodyModel)model.Tag)
		{
			_model = model;
			_meshTransforms = new Matrix[_model.Bones.Count];
			_model.CopyAbsoluteBoneTransformsTo(_meshTransforms);

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
			if (_isColorRandom = isColorRandom)
			{
				_diffuseColor = new Vector3((float)_colorRand.NextDouble(),
					(float)_colorRand.NextDouble(), (float)_colorRand.NextDouble());
				_diffuseColor *= 0.6f;
			}
		}

		public void Draw(IViewManager view)
		{
			if (Constants.DebugCollisions)
			{
				view.Device.RenderState.FillMode = FillMode.WireFrame;
				view.Device.RenderState.CullMode = CullMode.None;
			}
			else
			{
				view.Device.RenderState.FillMode = FillMode.Solid;
				view.Device.RenderState.CullMode = CullMode.CullClockwiseFace;
			}

			foreach (var mesh in _model.Meshes)
			{
				foreach (BasicEffect effect in mesh.Effects)
				{
					effect.World = _meshTransforms[mesh.ParentBone.Index] * Transform.Combined;
					effect.View = view.View;
					effect.Projection = view.Projection;
					if (_isColorRandom) effect.DiffuseColor = _diffuseColor;
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
