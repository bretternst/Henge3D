using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace Henge3D.Collision
{
	public class SphereCapsule : NarrowPhase
	{
		public override void OverlapTest(CollisionFunctor cf, Part partA, Part partB)
		{
			var a = (SpherePart)partA;
			var b = (CapsulePart)partB;

			Segment cap = new Segment(b.World.P1, b.World.P2);
			Vector3 pa, pb, normal, v;
			float r2 = a.World.Radius + b.World.Radius;
			r2 *= r2;

			float sb;
			cap.ClosestPointTo(ref a.World.Center, out sb, out pb);
			Vector3.Subtract(ref a.World.Center, ref pb, out normal);
			if (normal.LengthSquared() - r2 >= Constants.Epsilon)
				return;
			normal.Normalize();

			Vector3.Multiply(ref normal, -a.World.Radius, out v);
			Vector3.Add(ref a.World.Center, ref v, out pa);
			Vector3.Multiply(ref normal, b.World.Radius, out v);
			Vector3.Add(ref pb, ref v, out pb);
			cf.WritePoint(ref pa, ref pb, ref normal);
		}

		public override void SweptTest(CollisionFunctor cf, Part partA, Part partB, Vector3 delta)
		{
			var a = (SpherePart)partA;
			var b = (CapsulePart)partB;

			Segment path;
			path.P1 = a.World.Center;
			Vector3.Add(ref path.P1, ref delta, out path.P2);

			Capsule cap = b.World;
			cap.Radius += a.World.Radius;
			Segment capSegment = new Segment(b.World.P1, b.World.P2);

			float k;
			Vector3 pa, pb, normal;
			cap.Intersect(ref path, out k, out pa);
			if (k <= 1f)
			{
				capSegment.ClosestPointTo(ref pa, out k, out pb);
				Vector3.Subtract(ref pa, ref pb, out normal);
				normal.Normalize();
				Vector3.Multiply(ref normal, b.World.Radius, out pa);
				Vector3.Add(ref pb, ref pa, out pb);
				Vector3.Multiply(ref normal, -a.World.Radius, out pa);
				Vector3.Add(ref a.World.Center, ref pa, out pa);

				cf.WritePoint(ref pa, ref pb, ref normal);
			}
		}
	}
}
