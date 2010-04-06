using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace Henge3D.Collision
{
	public class CapsuleCapsule : NarrowPhase
	{
		public override void OverlapTest(CollisionFunctor cf, Part partA, Part partB)
		{
			var a = (CapsulePart)partA;
			var b = (CapsulePart)partB;

			DoOverlapTest(cf, a, b, Vector3.Zero);
		}

		public override void SweptTest(CollisionFunctor cf, Part partA, Part partB, Vector3 delta)
		{
			var a = (CapsulePart)partA;
			var b = (CapsulePart)partB;

			Vector3 step, offset = Vector3.Zero;
			int steps = (int)(delta.Length() / a.World.Radius * 0.9f);
			Vector3.Divide(ref delta, steps, out step);

			while (steps-- >= 0 && !DoOverlapTest(cf, a, b, offset))
			{
				Vector3.Add(ref offset, ref step, out offset);
			}
		}

		private static bool DoOverlapTest(CollisionFunctor cf, CapsulePart a, CapsulePart b, Vector3 offset)
		{
			Segment capa, capb = new Segment(b.World.P1, b.World.P2);
			Vector3.Add(ref a.World.P1, ref offset, out capa.P1);
			Vector3.Add(ref a.World.P2, ref offset, out capa.P2);

			Vector3 pa, pb, normal, v;
			float sa, sb, r2 = a.World.Radius + b.World.Radius;
			r2 *= r2;

			// find the closest point between the two capsules
			Segment.ClosestPoints(ref capa, ref capb, out sa, out pa, out sb, out pb);
			Vector3.Subtract(ref pa, ref pb, out normal);
			if (normal.LengthSquared() - r2 >= Constants.Epsilon)
				return false;
			if (normal.LengthSquared() < Constants.Epsilon)
				normal = Vector3.UnitZ;

			normal.Normalize();
			Vector3.Multiply(ref normal, -a.World.Radius, out v);
			Vector3.Add(ref pa, ref v, out pa);
			Vector3.Multiply(ref normal, b.World.Radius, out v);
			Vector3.Add(ref pb, ref v, out pb);
			Vector3.Subtract(ref pa, ref offset, out pa);
			cf.WritePoint(ref pa, ref pb, ref normal);

			// if the two capsules are nearly parallel, an additional support point provides stability
			if (sa == 0f || sa == 1f)
			{
				pa = sa == 0f ? capa.P2 : capa.P1;
				capb.ClosestPointTo(ref pa, out sa, out pb);
			}
			else if (sb == 0f || sb == 1f)
			{
				pb = sb == 0f ? capb.P2 : capb.P1;
				capa.ClosestPointTo(ref pb, out sb, out pa);
			}
			else
				return true;

			float dist;
			Vector3.DistanceSquared(ref pa, ref pb, out dist);
			if (dist - r2 < Constants.Epsilon)
			{
				Vector3.Multiply(ref normal, -a.World.Radius, out v);
				Vector3.Add(ref pa, ref v, out pa);
				Vector3.Multiply(ref normal, b.World.Radius, out v);
				Vector3.Add(ref pb, ref v, out pb);
				Vector3.Subtract(ref pa, ref offset, out pa);
				cf.WritePoint(ref pa, ref pb, ref normal);
			}
			return true;
		}
	}
}
