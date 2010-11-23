using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Henge3D.Physics;

namespace Henge3D.Holodeck
{
	public class Room : RigidBody, IVisible
	{
		#region Mesh

		private static VertexPositionNormalTexture[] _vertices = 
		{
			new VertexPositionNormalTexture(new Vector3(-1.0F, -1.0F, 0.0f), Vector3.UnitZ, new Vector2(0,20)), // B-1
			new VertexPositionNormalTexture(new Vector3(-1.0F, 1.0F, 0.0f), Vector3.UnitZ, new Vector2(20,20)), // B-2
			new VertexPositionNormalTexture(new Vector3(1.0F, 1.0F, 0.0f), Vector3.UnitZ, new Vector2(20,0)), // B-3
			new VertexPositionNormalTexture(new Vector3(1.0F, -1.0F, 0.0f), Vector3.UnitZ, new Vector2(0,0)), // B-4

			new VertexPositionNormalTexture(new Vector3(-1.0F, -1.0F, 1.0f), Vector3.UnitZ, new Vector2(0,20)), // T-1
			new VertexPositionNormalTexture(new Vector3(-1.0F, 1.0F, 1.0f), Vector3.UnitZ, new Vector2(20,20)), // T-2
			new VertexPositionNormalTexture(new Vector3(1.0F, 1.0F, 1.0f), Vector3.UnitZ, new Vector2(20,0)), // T-3
			new VertexPositionNormalTexture(new Vector3(1.0F, -1.0F, 1.0f), Vector3.UnitZ, new Vector2(0,0)), // T-4

			new VertexPositionNormalTexture(new Vector3(-1.0F, -1.0F, 1.0f), Vector3.UnitZ, new Vector2(0,0)), // T-1
			new VertexPositionNormalTexture(new Vector3(-1.0F, 1.0F, 1.0f), Vector3.UnitZ, new Vector2(20,0)), // T-2
			new VertexPositionNormalTexture(new Vector3(1.0F, 1.0F, 1.0f), Vector3.UnitZ, new Vector2(40,0)), // T-3
			new VertexPositionNormalTexture(new Vector3(1.0F, -1.0F, 1.0f), Vector3.UnitZ, new Vector2(60,0)), // T-4
			new VertexPositionNormalTexture(new Vector3(-1.0F, -1.0F, 0.0f), Vector3.UnitZ, new Vector2(0,10)), // B-1
			new VertexPositionNormalTexture(new Vector3(-1.0F, 1.0F, 0.0f), Vector3.UnitZ, new Vector2(20,10)), // B-2
			new VertexPositionNormalTexture(new Vector3(1.0F, 1.0F, 0.0f), Vector3.UnitZ, new Vector2(40,10)), // B-3
			new VertexPositionNormalTexture(new Vector3(1.0F, -1.0F, 0.0f), Vector3.UnitZ, new Vector2(60,10)), // B-4

			new VertexPositionNormalTexture(new Vector3(-1.0F, -1.0F, 1.0f), Vector3.UnitZ, new Vector2(80,0)), // T-1
			new VertexPositionNormalTexture(new Vector3(-1.0F, -1.0F, 0.0f), Vector3.UnitZ, new Vector2(80,10)), // B-1
		};

		static short[] _indices =
		{
			2, 1, 0, 0, 3, 2, // floor
			5, 6, 7, 7, 4, 5, // ceiling
			13, 9, 8, 8, 12, 13, // front
			14, 10, 9, 9, 13, 14, // right
			15, 11, 10, 10, 14, 15, // back
			17, 16, 11, 11, 15, 17 // left
		};

		#endregion

		private static float _scale = 10.0f;
		private static BasicEffect _effect;
		private static VertexDeclaration _vertexDeclaration;

		public Room(Game game)
		{
			this.Skin.DefaultMaterial = new Material(1f, 0.5f);
			this.Skin.Add(
				new PlanePart(new Vector3(0, 0, 1), -Vector3.UnitZ),
				new PlanePart(new Vector3(0, 0, 0), Vector3.UnitZ),
				new PlanePart(new Vector3(0, 1, 0), -Vector3.UnitY),
				new PlanePart(new Vector3(0, -1, 0), Vector3.UnitY),
				new PlanePart(new Vector3(1, 0, 0), -Vector3.UnitX),
				new PlanePart(new Vector3(-1, 0, 0), Vector3.UnitX)
				);
			this.SetWorld(_scale, Vector3.Zero, Quaternion.Identity);

			int ts = 32;
			Texture2D wallTexture = new Texture2D(game.GraphicsDevice, ts, ts);
			Color[] pixels = new Color[ts * ts];
			for (int i = 0; i < ts; i++)
			{
				pixels[i] = Color.Yellow;
				pixels[i * ts] = Color.Yellow;
				pixels[i * ts + (ts - 1)] = Color.Yellow;
				pixels[(ts - 1) * ts + i] = Color.Yellow;
			}
			wallTexture.SetData(pixels);

			_effect = new BasicEffect(game.GraphicsDevice);
			_vertexDeclaration = VertexPositionNormalTexture.VertexDeclaration;
			_effect.AmbientLightColor = Vector3.One;
			_effect.TextureEnabled = true;
			_effect.Texture = wallTexture;
		}

		public void Draw(IViewManager view)
		{
			_effect.View = view.View;
			_effect.World = Transform.Combined;
			_effect.Projection = view.Projection;
			foreach (EffectPass pass in _effect.CurrentTechnique.Passes)
			{
				pass.Apply();
				view.Device.DrawUserIndexedPrimitives(PrimitiveType.TriangleList,
					_vertices, 0, _vertices.Length, _indices, 0, _indices.Length / 3);
			}
		}
	}
}
