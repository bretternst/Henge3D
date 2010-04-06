using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace Henge3D.Collision
{
	public class SpherePolyhedron : NarrowPhase
	{
		public override void OverlapTest(CollisionFunctor cf, Part partA, Part partB)
		{
			var a = (SpherePart)partA;
			var b = (PolyhedronPart)partB;

			Vector3 pa, pb, normal, v;
			float ax, bx, r2 = a.World.Radius * a.World.Radius;
			float curDepth, finalDepth = float.MaxValue;
			int faceIdx = -1;

			for (int i = 0; i < b.FaceCount; i++)
			{
				b.World(b.Face(i)[0], out v);
				b.FaceNormal(i, out normal);
				Vector3.Dot(ref normal, ref v, out bx);
				Vector3.Dot(ref normal, ref a.World.Center, out ax);
				bx += a.World.Radius;
				curDepth = bx + a.World.Radius - ax;
				if (curDepth < 0f)
					return;
				if (curDepth < finalDepth)
				{
					faceIdx = i;
					finalDepth = curDepth;
				}
			}

			int[] face = b.Face(faceIdx);
			b.World(face[0], out v);
			b.FaceNormal(faceIdx, out normal);
			Plane plane = new Plane(v, normal);
			Vector3.Dot(ref normal, ref v, out bx);
			bx += a.World.Radius;

			plane.ClosestPointTo(ref a.World.Center, out pb);
			if (!b.IsPointOnFace(faceIdx, ref pb, true))
			{
				bool found = false;
				for (int i = 0; i < face.Length; i++)
				{
					float s;
					Segment edge;
					b.World(face[i == 0 ? face.Length - 1 : i - 1], out edge.P1);
					b.World(face[i], out edge.P2);
					edge.ClosestPointTo(ref a.World.Center, out s, out pb);
					Vector3.Subtract(ref a.World.Center, ref pb, out normal);
					if (normal.LengthSquared() - r2 < Constants.Epsilon)
					{
						normal.Normalize();
						found = true;
						break;
					}
				}
				if (!found)
					return;
			}

			Vector3.Multiply(ref normal, -a.World.Radius, out pa);
			Vector3.Add(ref a.World.Center, ref pa, out pa);
			cf.WritePoint(ref pa, ref pb, ref normal);
		}

		public override void SweptTest(CollisionFunctor cf, Part partA, Part partB, Vector3 delta)
		{
			var a = (SpherePart)partA;
			var b = (PolyhedronPart)partB;

			Vector3 p, pa, pb = Vector3.Zero, normal = Vector3.Zero;
			float scalar = float.PositiveInfinity, r2 = a.World.Radius * a.World.Radius;

			Segment path;
			path.P1 = a.World.Center;
			Vector3.Add(ref a.World.Center, ref delta, out path.P2);

			for (int i = 0; i < b.FaceCount; i++)
			{
				Vector3 n, cpa, cpb;
				b.World(b.Face(i)[0], out p);
				b.FaceNormal(i, out n);

				// only consider faces that the sphere is moving "toward"
				float ax, bx, dx;
				Vector3.Dot(ref n, ref delta, out dx);
				if (dx > 0f)
					continue;
				Vector3.Dot(ref n, ref p, out bx);
				Vector3.Dot(ref n, ref a.World.Center, out ax);
				if (ax < bx)
					continue;

				Plane plane;
				plane.Normal = n;

				Vector3.Multiply(ref n, a.World.Radius, out plane.P);
				Vector3.Add(ref p, ref plane.P, out plane.P);

				float scalarA, scalarB;
				if (plane.Intersect(ref path, out scalarB, out cpb))
				{
					if (b.IsPointOnFace(i, ref cpb, true))
					{
						normal = n;
						plane.P = p;
						plane.ClosestPointTo(ref cpb, out pb);
						Vector3.Multiply(ref normal, -a.World.Radius, out pa);
						Vector3.Add(ref a.World.Center, ref pa, out pa);
						cf.WritePoint(ref pa, ref pb, ref normal);
						return;
					}
				}

				int[] face = b.Face(i);
				for (int j = 0; j < face.Length; j++)
				{
					Segment edge;
					b.World(face[j == 0 ? face.Length - 1 : j - 1], out edge.P1);
					b.World(face[j], out edge.P2);
					Segment.ClosestPoints(ref path, ref edge, out scalarA, out cpa, out scalarB, out cpb);
					float dist;
					Vector3.DistanceSquared(ref cpa, ref cpb, out dist);
					if (dist - r2 < Constants.Epsilon)
					{
						if (Math.Abs(scalar - scalarA) < Constants.Epsilon)
						{
							Vector3.Add(ref normal, ref n, out normal);
						}
						else if (scalarA < scalar)
						{
							normal = n;
							pb = cpb;
							scalar = scalarA;
						}
					}
				}
			}

			if (scalar >= 0f && scalar <= 1f)
			{
				normal.Normalize();
				Vector3.Multiply(ref normal, -a.World.Radius, out pa);
				Vector3.Add(ref a.World.Center, ref pa, out pa);
				cf.WritePoint(ref pa, ref pb, ref normal);
			}
		}
	}
}
