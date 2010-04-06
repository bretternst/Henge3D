using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace Henge3D
{
	/// <summary>
	/// Represents a sphere, defined by a center point and a radius.
	/// </summary>
	public struct Sphere
	{
		public Vector3 Center;
		public float Radius;

		/// <summary>
		/// Construct a new sphere.
		/// </summary>
		/// <param name="center">The center of the sphere.</param>
		/// <param name="radius">The sphere's radius.</param>
		public Sphere(Vector3 center, float radius)
		{
			Center = center;
			Radius = radius;
		}

		/// <summary>
		/// Determines whether the sphere contains the point.
		/// </summary>
		/// <param name="p">The point to test for containment in the sphere.</param>
		/// <returns>Returns a value indicating whether the sphere contains the point.</returns>
		public bool Contains(ref Vector3 p)
		{
			float dist;
			Vector3.DistanceSquared(ref Center, ref p, out dist);
			return dist <= Radius * Radius;
		}

		/// <summary>
		/// Intersects a line segment with the shape and returns the intersection point closest to the beginning of the segment.
		/// </summary>
		/// <param name="segment">The line segment to intersect.</param>
		/// <param name="scalar">A value between 0 and 1 indicating how far along the segment the intersection occurs.</param>
		/// <param name="p">The point of the intersection closest to the beginning of the line segment.</param>
		/// <returns>Returns a value indicating whether there is an intersection.</returns>
		public bool Intersect(ref Segment seg, out float scalar, out Vector3 p)
		{
			p = Vector3.Zero;
			scalar = float.NaN;

			Vector3 s, d;
			Vector3.Subtract(ref Center, ref seg.P1, out s);
			Vector3.Subtract(ref seg.P2, ref seg.P1, out d);
			float l2s = Vector3.Dot(s, s);
			float r2 = Radius * Radius;

			if (l2s <= r2)
			{
				p = seg.P1;
				scalar = 0f;
				return true;
			}

			float tc = Vector3.Dot(s, d) / Vector3.Dot(d, d);
			float l2h = (r2 - l2s) / d.LengthSquared() + tc * tc;
			if (l2h < 0f) return false;

			scalar = tc - (float)Math.Sqrt(l2h);
			Vector3.Multiply(ref d, scalar, out p);
			Vector3.Add(ref seg.P1, ref p, out p);

			return scalar >= 0f && scalar <= 1f;
		}

		/// <summary>
		/// Attempts to encapsulate a cloud of points within a sphere. This may not produce an optimal or exact fit. The results
		/// are approximate.
		/// </summary>
		/// <param name="points">The list of points with which to build an enclosing sphere.</param>
		/// <param name="capsule">Returns the sphere containing all points.</param>
		public static void Fit(IList<Vector3> points, out Sphere sphere)
		{
			if (points.Count < 2)
			{
				sphere = new Sphere(points.Count == 0 ? Vector3.Zero : points[0], 0f);
				return;
			}

			var weights = new float[points.Count];
			var f = 1f / points.Count;
			for (int i = 0; i < weights.Length; i++)
				weights[i] = f;

			while (true)
			{
				sphere.Center = Vector3.Zero;
				for(int i = 0; i < points.Count; i++)
				{
					var p = points[i];
					Vector3.Multiply(ref p, weights[i], out p);
					Vector3.Add(ref sphere.Center, ref p, out sphere.Center);
				}

				float sumWeightedRadiusSq = 0f, sumWeightedRadius = 0f;
				sphere.Radius = 0f;
				for (int i = 0; i < points.Count; i++)
				{
					var p = points[i];
					float radiusSq;
					Vector3.DistanceSquared(ref sphere.Center, ref p, out radiusSq);
					sumWeightedRadiusSq += radiusSq * weights[i];
					float radius = (float)Math.Sqrt(radiusSq);
					if (sphere.Radius < radius)
						sphere.Radius = radius;
					weights[i] *= radius;
					sumWeightedRadius += weights[i];
				}
				if (sphere.Radius - (float)Math.Sqrt(sumWeightedRadiusSq) < sphere.Radius * Constants.Epsilon)
					break;
				for (int i = 0; i < points.Count; i++)
				{
					weights[i] /= sumWeightedRadius;
				}
			}
		}
	}
}
