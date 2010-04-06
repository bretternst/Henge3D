using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace Henge3D
{
	/// <summary>
	/// Represents a capsule, or a shape with a cylindrical mid-section and a hemisphere on each end. A capsule is defined by a
	/// line segment with a radius.
	/// </summary>
	public struct Capsule
	{
		public Vector3 P1, P2;
		public float Radius;

		/// <summary>
		/// Construct a capsule with the specified endpoints and radius. The points are the "center" of each hemisphere, not the very tips of
		/// the capsule.
		/// </summary>
		/// <param name="p1">The first segment endpoint.</param>
		/// <param name="p2">The second segment endpoint.</param>
		/// <param name="radius">The segment radius.</param>
		public Capsule(Vector3 p1, Vector3 p2, float radius)
		{
			P1 = p1;
			P2 = p2;
			Radius = radius;
		}

		/// <summary>
		/// Intersects a line segment with the shape and returns the intersection point closest to the beginning of the segment.
		/// </summary>
		/// <param name="segment">The line segment to intersect.</param>
		/// <param name="scalar">A value between 0 and 1 indicating how far along the segment the intersection occurs.</param>
		/// <param name="p">The point of the intersection closest to the beginning of the line segment.</param>
		/// <returns>Returns a value indicating whether there is an intersection.</returns>
		public bool Intersect(ref Segment segment, out float scalar, out Vector3 p)
		{
			Vector3 d, m, n;
			float md, nd, dd;
			Vector3.Subtract(ref P2, ref P1, out d);
			Vector3.Subtract(ref segment.P1, ref P1, out m);
			Vector3.Subtract(ref segment.P2, ref segment.P1, out n);
			Vector3.Dot(ref d, ref m, out md);
			Vector3.Dot(ref d, ref n, out nd);
			Vector3.Dot(ref d, ref d, out dd);

			if (!(md < 0f && md + nd < 0f) &&
				!(md > dd && md + nd > dd))
			{
				float nn, mn, k;
				Vector3.Dot(ref n, ref n, out nn);
				Vector3.Dot(ref m, ref n, out mn);
				float a = dd * nn - nd * nd;
				Vector3.Dot(ref m, ref m, out k);
				k -= Radius * Radius;
				float c = dd * k - md * md;
				if (Math.Abs(a) >= Constants.Epsilon)
				{
					float b = dd * mn - nd * md;
					float discr = b * b - a * c;
					if (discr >= 0f)
					{
						float t = (-b - (float)Math.Sqrt(discr)) / a;
						if (t >= 0f && t <= 1f &&
							md + t * nd >= 0f &&
							md + t * nd <= dd)
						{
							scalar = t;
							Vector3.Multiply(ref n, t, out p);
							Vector3.Add(ref segment.P1, ref p, out p);
							return true;
						}
					}
				}
			}

			var cap = new Sphere(P1, Radius);
			float s;
			scalar = float.MaxValue;
			p = Vector3.Zero;

			Vector3 v;
			if (cap.Intersect(ref segment, out s, out v))
			{
				scalar = s;
				p = v;
			}
			cap.Center = P2;
			if (cap.Intersect(ref segment, out s, out v) && s < scalar)
			{
				scalar = s;
				p = v;
			}

			return scalar >= 0f && scalar <= 1f;
		}

		/// <summary>
		/// Attempts to encapsulate a cloud of points within a capsule. This may not produce an optimal or exact fit. The results
		/// are approximate.
		/// </summary>
		/// <param name="points">The list of points with which to build an enclosing capsule.</param>
		/// <param name="capsule">Returns the capsule containing all points.</param>
		public static void Fit(IList<Vector3> points, out Capsule capsule)
		{
			capsule = new Capsule();
			Vector3 axis, axisNeg;
			GeometryHelper.ComputeMaxSpreadAxis(points, out axis);
			Vector3.Negate(ref axis, out axisNeg);
			if (axis.LengthSquared() < Constants.Epsilon)
				return;
			GeometryHelper.ComputeExtremePoint(points, ref axis, out capsule.P2);
			GeometryHelper.ComputeExtremePoint(points, ref axisNeg, out capsule.P1);
			Segment s = new Segment(capsule.P1, capsule.P2);
			for (int i = 0; i < points.Count; i++)
			{
				var p = points[i];
				float x = s.DistanceSquaredTo(ref p);
				if (x > capsule.Radius)
				{
					capsule.Radius = x;
				}
			}
			capsule.Radius = (float)Math.Sqrt(capsule.Radius);
			Vector3.Multiply(ref axis, capsule.Radius, out axis);
			Vector3.Negate(ref axis, out axisNeg);
			Vector3.Add(ref capsule.P1, ref axis, out capsule.P1);
			Vector3.Add(ref capsule.P2, ref axisNeg, out capsule.P2);
		}
	}
}
