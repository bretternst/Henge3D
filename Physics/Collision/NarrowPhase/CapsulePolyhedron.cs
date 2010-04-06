using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace Henge3D.Collision
{
	public class CapsulePolyhedron : NarrowPhase
	{
		public override void OverlapTest(CollisionFunctor cf, Part partA, Part partB)
		{
			var a = (CapsulePart)partA;
			var b = (PolyhedronPart)partB;

			DoOverlapTest(cf, a, b, Vector3.Zero);
		}

		public override void SweptTest(CollisionFunctor cf, Part partA, Part partB, Vector3 delta)
		{
			var a = (CapsulePart)partA;
			var b = (PolyhedronPart)partB;

			Vector3 step, offset = Vector3.Zero;
			int steps = (int)(delta.Length() / a.World.Radius * 0.9f);
			Vector3.Divide(ref delta, steps, out step);

			while (steps-- >= 0 && !DoOverlapTest(cf, a, b, offset))
			{
				Vector3.Add(ref offset, ref step, out offset);
			}
		}

		public static bool DoOverlapTest(CollisionFunctor cf, CapsulePart a, PolyhedronPart b, Vector3 offset)
		{
			Vector3 v, normal;
			float ax1, ax2, bx, r2 = a.World.Radius * a.World.Radius;

			Segment capa;
			Vector3.Add(ref a.World.P1, ref offset, out capa.P1);
			Vector3.Add(ref a.World.P2, ref offset, out capa.P2);

			float curDepth, finalDepth = float.MaxValue;
			int faceIdx = -1;

			// find the face with the least penetration depth along its normal
			for (int i = 0; i < b.FaceCount; i++)
			{
				b.World(b.Face(i)[0], out v);
				b.FaceNormal(i, out normal);
				Vector3.Dot(ref normal, ref v, out bx);
				Vector3.Dot(ref normal, ref capa.P1, out ax1);
				Vector3.Dot(ref normal, ref capa.P2, out ax2);
				bx += a.World.Radius;
				curDepth = Math.Max(bx - ax1, bx - ax2);
				if (curDepth < 0f)
					return false;
				if (curDepth < finalDepth)
				{
					faceIdx = i;
					finalDepth = curDepth;
				}
			}

			bool got1 = false, got2 = false;
			Vector3 pa, pb, pa1, pa2, pb1, pb2;
			pa1 = pa2 = pb1 = pb2 = Vector3.Zero;
			var face = b.Face(faceIdx);
			b.World(face[0], out v);
			b.FaceNormal(faceIdx, out normal);

			var plane = new Plane(v, normal);
			Vector3.Dot(ref normal, ref v, out bx);
			bx += a.World.Radius;

			// determine if either capsule point is inside the face
			pa1 = capa.P1;
			plane.ClosestPointTo(ref pa1, out pb1);
			Vector3.Dot(ref normal, ref pa1, out ax1);
			got1 = ax1 - bx < Constants.Epsilon && b.IsPointOnFace(faceIdx, ref pb1, true);

			pa2 = capa.P2;
			plane.ClosestPointTo(ref pa2, out pb2);
			Vector3.Dot(ref normal, ref pa2, out ax1);
			if (ax1 - bx < Constants.Epsilon && b.IsPointOnFace(faceIdx, ref pb2, true))
			{
				if (got1)
					got2 = true;
				else
				{
					got1 = true;
					pa1 = pa2;
					pb1 = pb2;
				}
			}

			// if one capsule point is inside the face but one is not, try to generate a second point on an edge
			if (got1 ^ got2)
			{
				float sbPrev = float.NaN;
				int edgePrev = -1;
				for (int i = 0; i < face.Length; i++)
				{
					float dist, sa, sb;
					Segment edge;
					b.World(face[i == 0 ? face.Length - 1 : i - 1], out edge.P1);
					b.World(face[i], out edge.P2);
					Segment.ClosestPoints(ref capa, ref edge, out sa, out pa, out sb, out pb);
					Vector3.DistanceSquared(ref pa, ref pb, out dist);
					if (dist - r2 < Constants.Epsilon && sa >= Constants.Epsilon && sa < 1 - Constants.Epsilon)
					{
						if (i == face.Length - 1 && edgePrev == 0 && sb == 1f) continue;
						if (!got2 || (edgePrev == i - 1 && sbPrev == 1f)) { pa2 = pa; pb2 = pb; got2 = true; }
						sbPrev = sb;
						edgePrev = i;
					}
				}
			}
			else if (!got1 && !got2)
			{
				// neither point is inside the face, so try all edges
				float sbPrev = float.NaN, edgePrev = float.NaN;
				for (int i = 0; i < face.Length; i++)
				{
					float dist, sa, sb;
					Segment edge;
					b.World(face[i == 0 ? face.Length - 1 : i - 1], out edge.P1);
					b.World(face[i], out edge.P2);
					Segment.ClosestPoints(ref capa, ref edge, out sa, out pa, out sb, out pb);
					Vector3.DistanceSquared(ref pa, ref pb, out dist);
					if (dist - r2 < Constants.Epsilon && sb > Constants.Epsilon)
					{
						if (i == face.Length - 1 && edgePrev == 0 && sb == 1f) continue;
						if (!got1 || (edgePrev == i - 1 && sbPrev == 1f)) { pa1 = pa; pb1 = pb; got1 = true; }
						else if (!got2) { pa2 = pa; pb2 = pb; got2 = true; }
						sbPrev = sb;
						edgePrev = i;
					}
				}

				// if only one edge was crossed, create a new normal instead of using the face normal
				if (got1 ^ got2)
				{
					Vector3.Subtract(ref pa1, ref pb1, out normal);
					normal.Normalize();
				}
			}

			// write out points
			if (got1 || got2)
			{
				Vector3.Multiply(ref normal, -a.World.Radius, out v);

				if (got1)
				{
					Vector3.Add(ref pa1, ref v, out pa1);
					Vector3.Subtract(ref pa1, ref offset, out pa1);
					cf.WritePoint(ref pa1, ref pb1, ref normal);
				}
				if (got2)
				{
					Vector3.Add(ref pa2, ref v, out pa2);
					Vector3.Subtract(ref pa2, ref offset, out pa2);
					cf.WritePoint(ref pa2, ref pb2, ref normal);
				}
			}

			return got1 || got2;
		}
	}
}
