using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

using Henge3D.Pipeline;

namespace Henge3D.Collision
{
	public class SphereMesh : NarrowPhase
	{
		public override void OverlapTest(CollisionFunctor cf, Part partA, Part partB)
		{
			var a = (SpherePart)partA;
			var b = (MeshPart)partB;

			var tf = OverlapFunctor;
			tf.Initialize(cf, a, b);
			b.ProcessTriangles(tf);

			// if the sphere is inside the mesh, push it out via the normal of least depth
			if (tf.Depth > 0f && tf.Depth < float.MaxValue)
			{
				Triangle tri;
				Vector3 pb;
				Triangle.Transform(ref tf.NearestTriangle, ref b.Transform, out tri);
				tri.Center(out pb);
				cf.WritePoint(ref a.World.Center, ref pb, ref tri.Normal);
			}
		}

		public override void SweptTest(CollisionFunctor cf, Part partA, Part partB, Vector3 delta)
		{
			var a = (SpherePart)partA;
			var b = (MeshPart)partB;

			var tf = SweptFunctor;
			tf.Initialize(cf, a, b, delta);
			b.ProcessTriangles(tf);
		}

		private class OverlapTriangleFunctor : TriangleFunctor
		{
			private CollisionFunctor _cf;
			private SpherePart _a;
			private MeshPart _b;
			private Vector3 _center;
			private float _radius;
			private float _radiusSquared;

			public float Depth;
			public Triangle NearestTriangle;

			public void Initialize(CollisionFunctor cf, SpherePart a, MeshPart b)
			{
				_cf = cf;
				_a = a;
				_b = b;
				_radius = a.World.Radius * b.TransformInverse.Scale;
				_radiusSquared = _radius * _radius;
				Depth = float.MaxValue;

				Vector3.Transform(ref a.World.Center, ref b.TransformInverse.Combined, out _center);
				BoundingBox.Minimum = BoundingBox.Maximum = _center;
				var radius = new Vector3(_radius);
				Vector3.Subtract(ref BoundingBox.Minimum, ref radius, out BoundingBox.Minimum);
				Vector3.Add(ref BoundingBox.Maximum, ref radius, out BoundingBox.Maximum);
			}

			public override void Process(int count)
			{
				Vector3 pa, pb, normal;
				float ax, bx;

				for (int i = 0; i < count; i++)
				{
					// record minimal depth and skip triangles that the sphere is behind
					Vector3.Dot(ref Buffer[i].Normal, ref _center, out ax);
					Vector3.Dot(ref Buffer[i].Normal, ref Buffer[i].V1, out bx);
					float d = bx - ax;
					if (d > 0f && d < Depth)
						this.NearestTriangle = Buffer[i];
					Depth = Math.Min(d, Depth);
					if (d > 0f)
						continue;

					bool useTriNormal = Buffer[i].ClosestPointTo(ref _center, out pb);
					Vector3.Subtract(ref _center, ref pb, out normal);
					if (normal.LengthSquared() - _radiusSquared >= Constants.Epsilon)
						continue;
					if (useTriNormal)
					{
						normal = Buffer[i].Normal;
					}
					else
					{
						normal.Normalize();
					}
					
					Vector3.Transform(ref normal, ref _b.Transform.Orientation, out normal);
					Vector3.Multiply(ref normal, -_a.World.Radius, out pa);
					Vector3.Add(ref _a.World.Center, ref pa, out pa);
					Vector3.Transform(ref pb, ref _b.Transform.Combined, out pb);

					_cf.WritePoint(ref pa, ref pb, ref normal);
				}
			}
		}

		private class SweptTriangleFunctor : TriangleFunctor
		{
			private CollisionFunctor _cf;
			private SpherePart _a;
			private MeshPart _b;
			private Segment _path;
			private float _radius;
			private float _radiusSquared;

			public void Initialize(CollisionFunctor cf, SpherePart a, MeshPart b, Vector3 delta)
			{
				_cf = cf;
				_a = a;
				_b = b;
				_radius = a.World.Radius * b.TransformInverse.Scale;
				_radiusSquared = _radius * _radius;

				Vector3.Transform(ref a.World.Center, ref b.TransformInverse.Combined, out _path.P1);
				Vector3.Transform(ref delta, ref b.TransformInverse.Orientation, out delta);
				Vector3.Multiply(ref delta, b.TransformInverse.Scale, out delta);
				Vector3.Add(ref _path.P1, ref delta, out _path.P2);

				AlignedBox.Fit(ref _path.P1, ref _path.P2, out BoundingBox);

				var radius = new Vector3(_radius);
				Vector3.Subtract(ref BoundingBox.Minimum, ref radius, out BoundingBox.Minimum);
				Vector3.Add(ref BoundingBox.Maximum, ref radius, out BoundingBox.Maximum);
			}

			public override void Process(int count)
			{
				Vector3 pa, pb, normal;
				float ax, bx;

				for (int i = 0; i < count; i++)
				{
					// skip triangles that the sphere is behind
					Vector3.Dot(ref Buffer[i].Normal, ref _path.P1, out ax);
					Vector3.Dot(ref Buffer[i].Normal, ref Buffer[i].V1, out bx);
					if (bx - ax > 0f)
						continue;

					float k;
					Buffer[i].ClosestPointTo(ref _path, out k, out pa, out pb);

					Vector3.Subtract(ref pa, ref pb, out normal);
					if (normal.LengthSquared() - _radiusSquared >= Constants.Epsilon)
						continue;
					normal = Buffer[i].Normal;

					Vector3.Transform(ref normal, ref _b.Transform.Orientation, out normal);
					Vector3.Multiply(ref normal, -_a.World.Radius, out pa);
					Vector3.Add(ref _a.World.Center, ref pa, out pa);
					Vector3.Transform(ref pb, ref _b.Transform.Combined, out pb);

					_cf.WritePoint(ref pa, ref pb, ref normal);
				}
			}
		}

		private static OverlapTriangleFunctor OverlapFunctor
		{
			get
			{
				var tf = _overlapFunctors[TaskManager.CurrentThreadIndex];
				if (tf == null)
					tf = _overlapFunctors[TaskManager.CurrentThreadIndex] = new OverlapTriangleFunctor();
				return tf;
			}
		}

		private static SweptTriangleFunctor SweptFunctor
		{
			get
			{
				var tf = _sweptFunctors[TaskManager.CurrentThreadIndex];
				if (tf == null)
					tf = _sweptFunctors[TaskManager.CurrentThreadIndex] = new SweptTriangleFunctor();
				return tf;
			}
		}

		private static OverlapTriangleFunctor[] _overlapFunctors = new OverlapTriangleFunctor[TaskManager.ThreadCount];
		private static SweptTriangleFunctor[] _sweptFunctors = new SweptTriangleFunctor[TaskManager.ThreadCount];
	}
}
