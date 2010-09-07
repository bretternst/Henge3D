using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

using Henge3D.Pipeline;

namespace Henge3D
{
	/// <summary>
	/// A mesh collision part, composed of triangles.
	/// </summary>
	public class MeshPart : Part
	{
		private CompiledMesh _body;
		private AlignedBox _boundingBox;

		public Transform Transform = Transform.Identity;
		public Transform TransformInverse = Transform.Identity;

		/// <summary>
		/// Construct a mesh collision part from a pre-compiled mesh.
		/// </summary>
		/// <param name="mesh">The compiled mesh that that this skin part will use.</param>
		public MeshPart(CompiledMesh mesh)
		{
			_body = mesh;
			ApplyTransform(ref Transform);
		}

		/// <summary>
		/// Construct a mesh collision part from the specified vertices and indices.
		/// </summary>
		/// <param name="vertices">A list of all vertices contained in the mesh.</param>
		/// <param name="indices">A list of vertex indices that define the triangles in the mesh.</param>
		public MeshPart(Vector3[] vertices, int[] indices)
		{
			_body = new CompiledMesh(vertices, indices);
			ApplyTransform(ref Transform);
		}

		/// <summary>
		/// Apply a transform to the collision skin part to bring it into world space.
		/// </summary>
		/// <param name="transform">The world-space transform to apply.</param>
		public override void ApplyTransform(ref Transform transform)
		{
			Transform = transform;
			transform.Invert(out TransformInverse);

			if (transform.Combined != Matrix.Identity)
			{
				_boundingBox = AlignedBox.Null;
				for (int i = 0; i < _body.Body.Length; i++)
				{
					Vector3 p;
					Vector3.Transform(ref _body.Body[i], ref transform.Combined, out p);
					_boundingBox.Add(ref p);
				}
			}
			else
				_boundingBox = _body.Nodes[0].BoundingBox;
		}

		/// <summary>
		/// Retrieves the bounding box enclosing the collision skin part.
		/// </summary>
		/// <param name="aabb">Returns the bounding box for this part.</param>
		public override void BoundingBox(out AlignedBox aabb)
		{
			aabb = _boundingBox;
		}

		#region Triangle Intersect Functor

		private class TriangleIntersectFunctor : TriangleFunctor
		{
			public Segment Segment;
			public float Scalar;
			public Vector3 Point;

			public override void Process(int count)
			{
				for (int i = 0; i < count; i++)
				{
					float s;
					Vector3 p;
					if (Buffer[i].Intersect(ref Segment, out s, out p) && s < Scalar)
					{
						Scalar = s;
						Point = p;
					}
				}
			}
		}

		private static TriangleIntersectFunctor[] _functors = new TriangleIntersectFunctor[TaskManager.ThreadCount];

		private static TriangleIntersectFunctor Functor
		{
			get
			{
				var tf = _functors[TaskManager.CurrentThreadIndex];
				if (tf == null)
					tf = _functors[TaskManager.CurrentThreadIndex] = new TriangleIntersectFunctor();
				return tf;
			}
		}

		#endregion

		/// <summary>
		/// Intersects a segment with this collision skin part and returns the intersection point that is nearest to the
		/// beginning of the segment.
		/// </summary>
		/// <param name="segment">The segment to intersect with.</param>
		/// <param name="scalar">Returns a value between 0 and 1 indicating where on the segment the first intersection occurs.</param>
		/// <param name="point">Returns the point of the first intersection.</param>
		/// <returns>Returns a value indicating whether the segment intersects with the part.</returns>
		public override bool Intersect(ref Segment segment, out float scalar, out Vector3 point)
		{
			Segment bodySeg;
			Segment.Transform(ref segment, ref TransformInverse, out bodySeg);

			AlignedBox box;
			AlignedBox.Fit(ref bodySeg.P1, ref bodySeg.P2, out box);

			var tf = Functor;
			tf.Segment = bodySeg;
			tf.Scalar = float.MaxValue;
			tf.Point = Vector3.Zero;
			tf.BoundingBox = box;

			_body.ProcessTriangles(tf);

			scalar = tf.Scalar;
			point = tf.Point;
			Vector3.Transform(ref point, ref Transform.Combined, out point);
			return scalar >= 0f && scalar <= 1f;
		}

		internal void ProcessTriangles(TriangleFunctor tf)
		{
			_body.ProcessTriangles(tf);
		}
	}
}
