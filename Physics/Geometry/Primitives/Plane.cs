using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace Henge3D
{
	/// <summary>
	/// Represents a plane, defined by a point intersecting the plane and a normal direction.
	/// </summary>
	public struct Plane
	{
		public Vector3 P;
		public Vector3 Normal;

		/// <summary>
		/// Construct a new plane.
		/// </summary>
		/// <param name="p">A point intersecting the plane.</param>
		/// <param name="normal">The normalized direction vector that points "up" away from the plane.</param>
		public Plane(Vector3 p, Vector3 normal)
		{
			P = p;
			Normal = normal;
		}

		/// <summary>
		/// Intersects a line with the shape and returns the intersection point, if there is one.
		/// </summary>
		/// <param name="segment">The line segment to intersect.</param>
		/// <param name="p">The point of the intersection.</param>
		/// <returns>Returns a value indicating whether there is an intersection.</returns>
		public bool Intersect(ref Line line, out Vector3 p)
		{
			p = Vector3.Zero;

			float a, d, e, t;
			Vector3.Dot(ref line.P, ref Normal, out a);
			Vector3.Dot(ref P, ref Normal, out d);
			Vector3.Dot(ref Normal, ref line.Direction, out e);
			if (Math.Abs(e) < Constants.Epsilon)
			{
				p = Vector3.Zero;
				return false;
			}
			else
			{
				t = (d - a) / e;
				Vector3.Multiply(ref line.Direction, t, out p);
				Vector3.Add(ref line.P, ref p, out p);
				return true;
			}
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
			p = Vector3.Zero;
			scalar = float.NaN;

			float a, d, e;
			Vector3 diff;
			Vector3.Subtract(ref segment.P2, ref segment.P1, out diff);
			Vector3.Dot(ref segment.P1, ref Normal, out a);
			Vector3.Dot(ref P, ref Normal, out d);
			Vector3.Dot(ref Normal, ref diff, out e);
			if (Math.Abs(e) <= Constants.Epsilon) return false;
			scalar = (d - a) / e;
			Vector3.Multiply(ref diff, scalar, out p);
			Vector3.Add(ref segment.P1, ref p, out p);
			return scalar >= 0f && scalar <= 1f;
		}

		/// <summary>
		/// Gets the closest point on the plane to the given point.
		/// </summary>
		/// <param name="p">The point to project onto the plane.</param>
		/// <param name="output">Returns the closest point on the plane.</param>
		public void ClosestPointTo(ref Vector3 p, out Vector3 output)
		{
			float dist;
			Vector3 tmp;
			Vector3.Subtract(ref p, ref P, out tmp);
			Vector3.Dot(ref Normal, ref tmp, out dist);
			Vector3.Multiply(ref Normal, dist, out tmp);
			Vector3.Subtract(ref p, ref tmp, out output);
		}
	}
}
