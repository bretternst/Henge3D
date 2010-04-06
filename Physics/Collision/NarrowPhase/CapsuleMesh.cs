using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

using Henge3D.Pipeline;

namespace Henge3D.Collision
{
	public class CapsuleMesh : NarrowPhase
	{
		public override void OverlapTest(CollisionFunctor cf, Part partA, Part partB)
		{
			var a = (CapsulePart)partA;
			var b = (MeshPart)partB;

			var tf = OverlapFunctor;
			tf.Initialize(cf, a, b, Vector3.Zero);
			b.ProcessTriangles(tf);
		}

		public override void SweptTest(CollisionFunctor cf, Part partA, Part partB, Vector3 delta)
		{
			var a = (CapsulePart)partA;
			var b = (MeshPart)partB;

			Vector3 step, offset = Vector3.Zero;
			int steps = (int)(delta.Length() / a.World.Radius * 0.9f);
			Vector3.Divide(ref delta, steps, out step);

			var tf = OverlapFunctor;
			while (steps-- >= 0)
			{
				tf.Initialize(cf, a, b, offset);
				b.ProcessTriangles(tf);
				if (tf.HasCollision)
					break;
				Vector3.Add(ref offset, ref step, out offset);
			}
		}

		private class OverlapTriangleFunctor : TriangleFunctor
		{
			private CollisionFunctor _cf;
			private MeshPart _b;
			private Segment _cap;
			private Vector3 _offset;
			private float _radius;
			private float _radiusSquared;
			private bool _hasCollision;

			public bool HasCollision { get { return _hasCollision; } }

			public void Initialize(CollisionFunctor cf, CapsulePart a, MeshPart b, Vector3 offset)
			{
				_cf = cf;
				_b = b;
				_radius = a.World.Radius * b.TransformInverse.Scale;
				_radiusSquared = _radius * _radius;
				_offset = offset;
				_hasCollision = false;
				Vector3.Add(ref a.World.P1, ref _offset, out _cap.P1);
				Vector3.Add(ref a.World.P2, ref _offset, out _cap.P2);

				// calculate points and bounding box in body space
				var radius = new Vector3(_radius);
				Vector3.Transform(ref _cap.P1, ref b.TransformInverse.Combined, out _cap.P1);
				Vector3.Transform(ref _cap.P2, ref b.TransformInverse.Combined, out _cap.P2);
				AlignedBox.Fit(ref _cap.P1, ref _cap.P2, out BoundingBox);
				Vector3.Subtract(ref BoundingBox.Minimum, ref radius, out BoundingBox.Minimum);
				Vector3.Add(ref BoundingBox.Maximum, ref radius, out BoundingBox.Maximum);
			}

			public override void Process(int count)
			{
				Vector3 pa, pb, normal, v;
				float dist, d;
				bool useTriNormal;

				for (int i = 0; i < count; i++)
				{
					// check capsule endpoints
					int ptCount = 0;
					for (int j = 0; j < 2; j++)
					{
						pa = j == 0 ? _cap.P1 : _cap.P2;
						useTriNormal = Buffer[i].ClosestPointTo(ref pa, out pb);
						Vector3.DistanceSquared(ref pa, ref pb, out dist);
						if (dist - _radiusSquared < Constants.Epsilon)
						{
							if (useTriNormal)
								normal = Buffer[i].Normal;
							else
							{
								// compute normal from triangle edge
								Vector3.Subtract(ref pa, ref pb, out normal);

								// if point is behind triangle, negate normal
								Vector3.Dot(ref normal, ref Buffer[i].Normal, out d);
								if (d <= -Constants.Epsilon)
								{
									Vector3.Multiply(ref Buffer[i].Normal, -2f * d, out v);
									Vector3.Add(ref normal, ref v, out normal);
								}
								normal.Normalize();
							}

							Vector3.Transform(ref normal, ref _b.Transform.Orientation, out normal);
							Vector3.Transform(ref pa, ref _b.Transform.Combined, out pa);
							Vector3.Transform(ref pb, ref _b.Transform.Combined, out pb);
							Vector3.Multiply(ref normal, -_radius, out v);
							Vector3.Add(ref pa, ref v, out pa);
							Vector3.Subtract(ref pa, ref _offset, out pa);
							_hasCollision = true;
							_cf.WritePoint(ref pa, ref pb, ref normal);
							ptCount++;
						}
					}

					if (ptCount > 1) continue;

					// check for capsule/edge intersection
					useTriNormal = Buffer[i].ClosestPointTo(ref _cap, out d, out pa, out pb);
					Vector3.DistanceSquared(ref pa, ref pb, out dist);
					if (d >= Constants.Epsilon && d <= 1f - Constants.Epsilon && dist - _radiusSquared < Constants.Epsilon)
					{
						if (useTriNormal)
							normal = Buffer[i].Normal;
						else
						{
							// compute normal from triangle edge
							Vector3.Subtract(ref pa, ref pb, out normal);

							// if point is behind triangle, negate normal
							Vector3.Dot(ref normal, ref Buffer[i].Normal, out d);
							if (d <= -Constants.Epsilon)
							{
								Vector3.Multiply(ref Buffer[i].Normal, -2f * d, out v);
								Vector3.Add(ref normal, ref v, out normal);
							}
							normal.Normalize();
						}

						// transform points back into world space
						Vector3.Transform(ref normal, ref _b.Transform.Orientation, out normal);
						Vector3.Transform(ref pa, ref _b.Transform.Combined, out pa);
						Vector3.Transform(ref pb, ref _b.Transform.Combined, out pb);
						Vector3.Multiply(ref normal, -_radius, out v);
						Vector3.Add(ref pa, ref v, out pa);
						Vector3.Subtract(ref pa, ref _offset, out pa);
						_cf.WritePoint(ref pa, ref pb, ref normal);
						_hasCollision = true;
					}
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

		private static OverlapTriangleFunctor[] _overlapFunctors = new OverlapTriangleFunctor[TaskManager.ThreadCount];
	}
}
