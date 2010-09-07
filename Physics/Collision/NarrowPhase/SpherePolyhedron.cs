using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace Henge3D.Collision
{
	public class SpherePolyhedron : NarrowPhase
	{
		public static bool DoOverlapTest(CollisionFunctor cf, SpherePart a, PolyhedronPart b, Vector3 offset)
		{
			Vector3 pa, pb, normal, v, center;
			float depth, r2 = a.World.Radius * a.World.Radius;
			bool foundAny = false;

			for (int i = 0; i < b.FaceCount; i++)
			{
				b.World(b.Face(i)[0], out v);
				b.FaceNormal(i, out normal);
				var plane = new Plane(v, normal);

				Vector3.Add(ref a.World.Center, ref offset, out center);
				Vector3.Subtract(ref center, ref v, out v);
				Vector3.Dot(ref normal, ref v, out depth);

				if (depth < 0f || depth - a.World.Radius >= Constants.Epsilon)
					continue;

				plane.ClosestPointTo(ref center, out pb);
				if (b.IsPointOnFace(i, ref pb, true))
				{
					Vector3.Subtract(ref pb, ref center, out v);
					if (v.LengthSquared() - r2 < Constants.Epsilon)
					{
						Vector3.Multiply(ref normal, -a.World.Radius, out pa);
						Vector3.Add(ref a.World.Center, ref pa, out pa);
						cf.WritePoint(ref pa, ref pb, ref normal);
						return true;
					}
				}
				else
				{
					int[] face = b.Face(i);
					for (int j = 0; j < face.Length; j++)
					{
						float s;
						Segment edge;
						b.World(face[j == 0 ? face.Length - 1 : j - 1], out edge.P1);
						b.World(face[j], out edge.P2);
						edge.ClosestPointTo(ref center, out s, out pb);
						Vector3.Subtract(ref center, ref pb, out normal);
						if (normal.LengthSquared() - r2 < Constants.Epsilon)
						{
							normal.Normalize();
							Vector3.Multiply(ref normal, -a.World.Radius, out pa);
							Vector3.Add(ref a.World.Center, ref pa, out pa);
							cf.WritePoint(ref pa, ref pb, ref normal);
							foundAny = true;
						}
					}
				}
			}
			return foundAny;
		}

		public override void OverlapTest(CollisionFunctor cf, Part partA, Part partB)
		{
			var a = (SpherePart)partA;
			var b = (PolyhedronPart)partB;

			DoOverlapTest(cf, a, b, Vector3.Zero);
		}

		public override void SweptTest(CollisionFunctor cf, Part partA, Part partB, Vector3 delta)
		{
			var a = (SpherePart)partA;
			var b = (PolyhedronPart)partB;

			Vector3 step, offset = Vector3.Zero;
			int steps = (int)(delta.Length() / a.World.Radius * 0.9f);
			Vector3.Divide(ref delta, steps, out step);

			while (steps-- >= 0 && !DoOverlapTest(cf, a, b, offset))
			{
				Vector3.Add(ref offset, ref step, out offset);
			}
		}
	}
}
