using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace Henge3D
{
	/// <summary>
	/// A plane collision skin part that is composed of an infinite plane that intersects a given point and has a given normal.
	/// Objects will collide with the plane whenever they intersect with or are "below" the plane relative to the normal.
	/// </summary>
	public sealed class PlanePart : Part
	{
		public Plane Plane;

		private Vector3 _point;
		private Vector3 _normal;

		/// <summary>
		/// Gets a point with which the plane intersects.
		/// </summary>
		public Vector3 Point { get { return _point; } }

		/// <summary>
		/// Gets the plane's normal.
		/// </summary>
		public Vector3 Normal { get { return _normal; } }

		/// <summary>
		/// Construct a collision part from a plane.
		/// </summary>
		/// <param name="p">A point with which the plane intersects.</param>
		/// <param name="normal">The plane normal.</param>
		public PlanePart(Vector3 p, Vector3 normal)
		{
			this._point = p;
			this._normal = normal;
			Plane = new Plane(p, normal);
		}

		/// <summary>
		/// Apply a transform to the collision skin part to bring it into world space.
		/// </summary>
		/// <param name="transform">The world-space transform to apply.</param>
		public override void ApplyTransform(ref Transform transform)
		{
			Vector3.Transform(ref _point, ref transform.Combined, out Plane.P);
			Vector3.Transform(ref _normal, ref transform.Orientation, out Plane.Normal);
		}

		/// <summary>
		/// Retrieves the bounding box enclosing the collision skin part.
		/// </summary>
		/// <param name="aabb">Returns the bounding box for this part.</param>
		public override void BoundingBox(out AlignedBox aabb)
		{
			aabb.Minimum = new Vector3(float.NegativeInfinity, float.NegativeInfinity, float.NegativeInfinity);
			aabb.Maximum = new Vector3(float.PositiveInfinity, float.PositiveInfinity, float.PositiveInfinity);
			if (_normal.X == 1f) aabb.Minimum.X = _point.X;
			else if (_normal.X == -1f) aabb.Maximum.X = _point.X;
			else if (_normal.Y == 1f) aabb.Minimum.Y = _point.Y;
			else if (_normal.Y == -1f) aabb.Maximum.Y = _point.Y;
			else if (_normal.Z == 1f) aabb.Minimum.Z = _point.Z;
			else if (_normal.Z == -1f) aabb.Maximum.Z = _point.Z;
		}

		/// <summary>
		/// Intersects a segment with this collision skin part and returns the intersection point that is nearest to the
		/// beginning of the segment.
		/// </summary>
		/// <param name="segment">The segment to intersect with.</param>
		/// <param name="scalar">Returns a value between 0 and 1 indicating where on the segment the first intersection occurs.</param>
		/// <param name="point">Returns the point of the first intersection.</param>
		/// <returns>Returns a value indicating whether the segment intersects with the part.</returns>
		public override bool Intersect(ref Segment segment, out float scalar, out Vector3 point)
		{
			return Plane.Intersect(ref segment, out scalar, out point);
		}
	}
}
