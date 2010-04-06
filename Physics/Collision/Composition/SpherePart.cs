using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace Henge3D
{
	/// <summary>
	/// A sphere collision part, defined by a center point and a radius.
	/// </summary>
	public class SpherePart : Part
	{
		public Sphere World;
		private Sphere _body;

		/// <summary>
		/// Construct the collision part from a sphere.
		/// </summary>
		/// <param name="sphere">The sphere defining the center and radius of the part.</param>
		public SpherePart(Sphere sphere)
		{
			_body = World = sphere;
		}

		/// <summary>
		/// Apply a transform to the collision skin part to bring it into world space.
		/// </summary>
		/// <param name="transform">The world-space transform to apply.</param>
		public override void ApplyTransform(ref Transform transform)
		{
			Vector3.Transform(ref _body.Center, ref transform.Combined, out World.Center);
		}

		/// <summary>
		/// Retrieves the bounding box enclosing the collision skin part.
		/// </summary>
		/// <param name="aabb">Returns the bounding box for this part.</param>
		public override void BoundingBox(out AlignedBox aabb)
		{
			aabb.Minimum = new Vector3(World.Center.X - _body.Radius, World.Center.Y - _body.Radius, World.Center.Z - _body.Radius);
			aabb.Maximum = new Vector3(World.Center.X + _body.Radius, World.Center.Y + _body.Radius, World.Center.Z + _body.Radius);
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
			return World.Intersect(ref segment, out scalar, out point);
		}
	}
}
