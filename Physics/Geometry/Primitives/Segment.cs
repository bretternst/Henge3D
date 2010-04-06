using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace Henge3D
{
	/// <summary>
	/// Represents a line segment as defined by two distinct endpoints.
	/// </summary>
	public struct Segment
	{
		public Vector3 P1;
		public Vector3 P2;

		/// <summary>
		/// Construct a new line segment.
		/// </summary>
		/// <param name="p1">The first endpoint.</param>
		/// <param name="p2">The second endpoint.</param>
		public Segment(Vector3 p1, Vector3 p2)
		{
			P1 = p1;
			P2 = p2;
		}

		/// <summary>
		/// Gets the squared distance between the given point and the segment.
		/// </summary>
		/// <param name="p">The point with which to calculate the distance.</param>
		/// <returns>Returns the squared distance.</returns>
		public float DistanceSquaredTo(ref Vector3 p)
		{
			Vector3 pq, pp, qp;
			float e, f;

			Vector3.Subtract(ref P2, ref P1, out pq);
			Vector3.Subtract(ref p, ref P1, out pp);
			Vector3.Subtract(ref p, ref P2, out qp);
			Vector3.Dot(ref pp, ref pq, out e);
			if (e <= 0f) return pp.LengthSquared();
			Vector3.Dot(ref pq, ref pq, out f);
			if (e >= f) return qp.LengthSquared();
			return pp.LengthSquared() - e * e / f;
		}

		/// <summary>
		/// Gets the closest point on the segment to the given point.
		/// </summary>
		/// <param name="p">The point with which to calculate the nearest segment point.</param>
		/// <param name="scalar">Returns a value between 0 and 1 indicating the location on the segment nearest the point.</param>
		/// <param name="output">Returns the closest point on the segment.</param>
		public void ClosestPointTo(ref Vector3 p, out float scalar, out Vector3 output)
		{
			Vector3 u, v;
			Vector3.Subtract(ref p, ref P1, out v);
			Vector3.Subtract(ref P2, ref P1, out u);
			Vector3.Dot(ref u, ref v, out scalar);
			scalar /= u.LengthSquared();
			if (scalar <= 0f)
				output = P1;
			else if (scalar >= 1f)
				output = P2;
			else
			{
				Vector3.Multiply(ref u, scalar, out output);
				Vector3.Add(ref P1, ref output, out output);
			}
		}

		/// <summary>
		/// Transform the segment using the specified transformation.
		/// </summary>
		/// <param name="s">The segment to transform.</param>
		/// <param name="transform">the transform to apply.</param>
		/// <param name="output">Returns the transformed segment.</param>
		public static void Transform(ref Segment s, ref Transform transform, out Segment output)
		{
			Vector3.Transform(ref s.P1, ref transform.Combined, out output.P1);
			Vector3.Transform(ref s.P2, ref transform.Combined, out output.P2);
		}

		/// <summary>
		/// Intersects two line segments.
		/// </summary>
		/// <param name="sa">The first line segment.</param>
		/// <param name="sb">The second line segment.</param>
		/// <param name="scalarA">Returns a value between 0 and 1 indicating the point of intersection on the first segment.</param>
		/// <param name="scalarB">Returns a value between 0 and 1 indicating the point of intersection on the second segment.</param>
		/// <param name="p">Returns the point of intersection, common to both segments.</param>
		/// <returns>Returns a value indicating whether there was an intersection.</returns>
		public static bool Intersect(ref Segment sa, ref Segment sb, out float scalarA, out float scalarB, out Vector3 p)
		{
			Vector3 pa;
			float dist;
			Segment.ClosestPoints(ref sa, ref sb, out scalarA, out pa, out scalarB, out p);
			Vector3.DistanceSquared(ref pa, ref p, out dist);
			return dist < Constants.Epsilon;
		}

		/// <summary>
		/// Gets the closest two points on two line segments.
		/// </summary>
		/// <remarks>
		/// If the line segments are parallel and overlap in their common direction, then the midpoint of the overlapped portion of line segments
		/// is returned.
		/// </remarks>
		/// <param name="sa">The first line segment.</param>
		/// <param name="sb">The second line segment.</param>
		/// <param name="scalarA">Returns a value between 0 and 1 indicating the position of the closest point on the first segment.</param>
		/// <param name="pa">Returns the closest point on the first segment.</param>
		/// <param name="scalarB">Returns a value between 0 and 1 indicating the position of the closest point on the second segment.</param>
		/// <param name="pb">Returns the closest point on the second segment.</param>
		public static void ClosestPoints(ref Segment sa, ref Segment sb,
			out float scalarA, out Vector3 pa, out float scalarB, out Vector3 pb)
		{
			Vector3 d1, d2, r;
			Vector3.Subtract(ref sa.P2, ref sa.P1, out d1);
			Vector3.Subtract(ref sb.P2, ref sb.P1, out d2);
			Vector3.Subtract(ref sa.P1, ref sb.P1, out r);
			float a, e, f;
			Vector3.Dot(ref d1, ref d1, out a);
			Vector3.Dot(ref d2, ref d2, out e);
			Vector3.Dot(ref d2, ref r, out f);

			if (a < Constants.Epsilon && e < Constants.Epsilon)
			{
				// segment a and b are both points
				scalarA = scalarB = 0f;
				pa = sa.P1;
				pb = sb.P1;
				return;
			}

			if (a < Constants.Epsilon)
			{
				// segment a is a point
				scalarA = 0f;
				scalarB = MathHelper.Clamp(f / e, 0f, 1f);
			}
			else
			{
				float c;
				Vector3.Dot(ref d1, ref r, out c);

				if (e < Constants.Epsilon)
				{
					// segment b is a point
					scalarB = 0f;
					scalarA = MathHelper.Clamp(-c / a, 0f, 1f);
				}
				else
				{
					float b;
					Vector3.Dot(ref d1, ref d2, out b);
					float denom = a * e - b * b;

					if (denom < Constants.Epsilon)
					{
						// segments are parallel
						float a1, a2, b1, b2;
						Vector3.Dot(ref d2, ref sa.P1, out a1);
						Vector3.Dot(ref d2, ref sa.P2, out a2);
						Vector3.Dot(ref d2, ref sb.P1, out b1);
						Vector3.Dot(ref d2, ref sb.P2, out b2);
						if (a1 <= b1 && a2 <= b1)
						{
							// segment A is completely "before" segment B
							scalarA = a2 > a1 ? 1f : 0f;
							scalarB = 0f;
						}
						else if (a1 >= b2 && a2 >= b2)
						{
							// segment B is completely "before" segment A
							scalarA = a2 > a1 ? 0f : 1f;
							scalarB = 1f;
						}
						else
						{
							// segments A and B overlap, use midpoint of shared length
							if (a1 > a2) { f = a1; a1 = a2; a2 = f; }
							f = (Math.Min(a2, b2) + Math.Max(a1, b1)) / 2f;
							scalarB = (f - b1) / e;
							Vector3.Multiply(ref d2, scalarB, out pb);
							Vector3.Add(ref sb.P1, ref pb, out pb);
							sa.ClosestPointTo(ref pb, out scalarA, out pa);
							return;
						}
					}
					else
					{
						// general case
						scalarA = MathHelper.Clamp((b * f - c * e) / denom, 0f, 1f);
						scalarB = (b * scalarA + f) / e;
						if (scalarB < 0f)
						{
							scalarB = 0f;
							scalarA = MathHelper.Clamp(-c / a, 0f, 1f);
						}
						else if (scalarB > 1f)
						{
							scalarB = 1f;
							scalarA = MathHelper.Clamp((b - c) / a, 0f, 1f);
						}
					}
				}
			}
			Vector3.Multiply(ref d1, scalarA, out d1);
			Vector3.Multiply(ref d2, scalarB, out d2);
			Vector3.Add(ref sa.P1, ref d1, out pa);
			Vector3.Add(ref sb.P1, ref d2, out pb);
		}
	}
}
