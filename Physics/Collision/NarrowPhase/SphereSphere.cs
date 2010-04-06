using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace Henge3D.Collision
{
	public class SphereSphere : NarrowPhase
	{
		public override void OverlapTest(CollisionFunctor cf, Part partA, Part partB)
		{
			var a = (SpherePart)partA;
			var b = (SpherePart)partB;

			float r2 = a.World.Radius + b.World.Radius;
			r2 *= r2;

			Vector3 pa, pb, normal;
			Vector3.Subtract(ref a.World.Center, ref b.World.Center, out normal);
			if (normal.LengthSquared() - r2 >= Constants.Epsilon)
				return;

			normal.Normalize();
			Vector3.Multiply(ref normal, -a.World.Radius, out pa);
			Vector3.Add(ref a.World.Center, ref pa, out pa);
			Vector3.Multiply(ref normal, b.World.Radius, out pb);
			Vector3.Add(ref b.World.Center, ref pb, out pb);
			cf.WritePoint(ref pa, ref pb, ref normal);
		}

		public override void SweptTest(CollisionFunctor cf, Part partA, Part partB, Vector3 delta)
		{
			var a = (SpherePart)partA;
			var b = (SpherePart)partB;

			float d2 = a.World.Radius + b.World.Radius;
			d2 *= d2;
			Vector3 v, p = b.World.Center, q = a.World.Center;
			Vector3.Add(ref q, ref delta, out v);
			Vector3.Subtract(ref v, ref q, out v);

			Vector3 pq;
			Vector3.Subtract(ref q, ref p, out pq);
			float ax, bx, cx;
			Vector3.Dot(ref v, ref v, out ax);
			Vector3.Dot(ref v, ref pq, out bx);
			Vector3.Dot(ref pq, ref pq, out cx);
			cx -= d2;

			float n = (-bx - (float)Math.Sqrt(bx * bx - ax * cx)) / ax;

			if (n <= 1f)
			{
				Vector3.Multiply(ref v, n, out q);
				Vector3.Add(ref a.World.Center, ref q, out q);
				Vector3 normal;
				Vector3.Subtract(ref q, ref p, out normal);
				normal.Normalize();

				Vector3 pa, pb;
				Vector3.Multiply(ref normal, -a.World.Radius, out pa);
				Vector3.Add(ref a.World.Center, ref pa, out pa);
				Vector3.Multiply(ref normal, b.World.Radius, out pb);
				Vector3.Add(ref b.World.Center, ref pb, out pb);

				cf.WritePoint(ref pa, ref pb, ref normal);
			}

			//Segment path;
			//path.P1 = a.World.Center;
			//Vector3.Add(ref path.P1, ref delta, out path.P2);

			//Vector3.Subtract(ref path.P2, ref path.P1, out v);
			//v.Normalize();
			//q = path.P1;


			//float scalar;
			//Vector3 normal, pa, pb;
			//path.ClosestPointTo(ref b.World.Center, out scalar, out pa);
			//Vector3.Subtract(ref pa, ref b.World.Center, out normal);

			//if (normal.LengthSquared() - r2 < Constants.Epsilon)
			//{
			//    normal.Normalize();
			//    Vector3.Multiply(ref normal, -a.World.Radius, out pa);
			//    Vector3.Add(ref a.World.Center, ref pa, out pa);
			//    Vector3.Multiply(ref normal, b.World.Radius, out pb);
			//    Vector3.Add(ref b.World.Center, ref pb, out pb);

			//    cf.WritePoint(ref pa, ref pb, ref normal);
			//}
		}
	}
}
