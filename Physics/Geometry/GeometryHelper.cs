using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace Henge3D
{
	/// <summary>
	/// Provides utility functions to support geometric operations used mainly in collision detection.
	/// </summary>
	public static class GeometryHelper
	{
		/// <summary>
		/// Compute the signed angle between two vector directions, rotated counter-clockwise around an axis.
		/// </summary>
		/// <param name="d1">The first normalized direction vector.</param>
		/// <param name="d2">The second normalized direction vector.</param>
		/// <param name="axis">The axis around which to compute the angle difference.</param>
		/// <returns>Returns the angle difference in radians.</returns>
		public static float ComputeSignedAngle(ref Vector3 d1, ref Vector3 d2, ref Vector3 axis)
		{
			float cos, sign;
			Vector3 n;
			Vector3.Cross(ref d1, ref d2, out n);
			Vector3.Dot(ref n, ref axis, out sign);

			Vector3.Dot(ref d1, ref d2, out cos);
			float d = (float)Math.Acos(cos);
			return sign >= 0f ? d : -d;
		}

		/// <summary>
		/// Determines whether the order of the specified vertices is counter-clockwise with respect to a normal.
		/// </summary>
		/// <param name="normal">The triangle normal.</param>
		/// <param name="p1">The first vertex.</param>
		/// <param name="p2">The second vertex.</param>
		/// <param name="p3">The third vertex.</param>
		/// <returns>Returns a value indicating whether the vertices are specified in counter-clockwise order.</returns>
		public static bool IsTriangleCcw(ref Vector3 normal, ref Vector3 p1, ref Vector3 p2, ref Vector3 p3)
		{
			Vector3 c1 = new Vector3(p1.X - p2.X, p1.Y - p2.Y, p1.Z - p2.Z);
			Vector3 c2 = new Vector3(p1.X - p3.X, p1.Y - p3.Y, p1.Z - p3.Z);
			Vector3.Cross(ref c1, ref c2, out c1);
			float r;
			Vector3.Dot(ref normal, ref c1, out r);
			return r >= Constants.Epsilon;
		}

		/// <summary>
		/// Computes the centroid, or the average, from a list of points.
		/// </summary>
		/// <remarks>
		/// Duplicate points are counted twice.
		/// </remarks>
		/// <param name="points">The list of input points.</param>
		/// <param name="centroid">Returns the centroid point.</param>
		public static void ComputeCentroid(IList<Vector3> points, out Vector3 centroid)
		{
			float denom = 1f / points.Count;
			centroid = Vector3.Zero;
			for (int i = 0; i < points.Count; i++)
			{
				var p = points[i];
				Vector3.Add(ref centroid, ref p, out centroid);
			}
			Vector3.Multiply(ref centroid, denom, out centroid);
		}

		/// <summary>
		/// Computes the most extreme point along a given direction from a list of points.
		/// </summary>
		/// <param name="points">The list of input points.</param>
		/// <param name="direction">The normalized direction along which to find an extreme.</param>
		/// <param name="output">Returns the most extreme point.</param>
		public static void ComputeExtremePoint(IList<Vector3> points, ref Vector3 direction, out Vector3 output)
		{
			float maxDist = float.NegativeInfinity;
			output = Vector3.Zero;
			for (int i = 0; i < points.Count; i++)
			{
				var p = points[i];
				float x;
				Vector3.Dot(ref direction, ref p, out x);
				if (x > maxDist)
				{
					maxDist = x;
					output = p;
				}
			}
		}

		/// <summary>
		/// Compute a covariance matrix from a list of points.
		/// </summary>
		/// <param name="points">The list of input points.</param>
		/// <param name="output">Returns the computed covariance matrix.</param>
		public static void ComputeCovarianceMatrix(IList<Vector3> points, out Matrix output)
		{
			float denom = 1f / points.Count;
			Vector3 center;
			ComputeCentroid(points, out center);
			output = new Matrix();

			for (int i = 0; i < points.Count; i++)
			{
				var p = points[i];
				Vector3.Subtract(ref p, ref center, out p);
				output.M11 += p.X * p.X;
				output.M22 += p.Y * p.Y;
				output.M33 += p.Z * p.Z;
				output.M12 += p.X * p.Y;
				output.M13 += p.X * p.Z;
				output.M23 += p.Y * p.Z;
			}

			output.M11 *= denom;
			output.M22 *= denom;
			output.M33 *= denom;
			output.M12 = output.M21 = output.M12 * denom;
			output.M13 = output.M31 = output.M13 * denom;
			output.M23 = output.M32 = output.M23 * denom;
		}

		/// <summary>
		/// Computes the axis of maximum spread from a list of points.
		/// </summary>
		/// <param name="points">The list of input points.</param>
		/// <param name="axis">Returns the axis of maximum spread.</param>
		public static void ComputeMaxSpreadAxis(IList<Vector3> points, out Vector3 axis)
		{
			Matrix co;
			ComputeCovarianceMatrix(points, out co);
			Vector3 b = new Vector3(0.5f,0.5f,0.5f), tmp = Vector3.Zero;

			for (int iter = 0; iter < 1000; iter++)
			{
				tmp.X += b.X * co.M11;
				tmp.X += b.Y * co.M21;
				tmp.X += b.Z * co.M31;
				tmp.Y += b.X * co.M12;
				tmp.Y += b.Y * co.M22;
				tmp.Y += b.Z * co.M32;
				tmp.Z += b.X * co.M13;
				tmp.Z += b.Y * co.M23;
				tmp.Z += b.Z * co.M33;
				tmp.Normalize();
				if (Math.Abs(b.X - tmp.X) < Constants.Epsilon &&
					Math.Abs(b.Y - tmp.Y) < Constants.Epsilon &&
					Math.Abs(b.Z - tmp.Z) < Constants.Epsilon)
					break;
				b = tmp;
			}
			b.Normalize();
			axis = b;
		}
	}
}
