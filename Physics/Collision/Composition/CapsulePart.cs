using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace Henge3D
{
	/// <summary>
	/// A capsule collision skin part that is composed of a cylindrical mid-section with a hemisphere cap on each end.
	/// </summary>
	public class CapsulePart : Part
	{
		public Capsule World;
		private Capsule _body;

		/// <summary>
		/// Constructs a collision part from a capsule.
		/// </summary>
		/// <param name="capsule">The capsule definition that this skin part will use.</param>
		public CapsulePart(Capsule capsule)
		{
			_body = World = capsule;
		}

		/// <summary>
		/// Apply a transform to the collision skin part to bring it into world space.
		/// </summary>
		/// <param name="transform">The world-space transform to apply.</param>
		public override void ApplyTransform(ref Transform transform)
		{
			Vector3.Transform(ref _body.P1, ref transform.Combined, out World.P1);
			Vector3.Transform(ref _body.P2, ref transform.Combined, out World.P2);
		}

		/// <summary>
		/// Retrieves the bounding box enclosing the collision skin part.
		/// </summary>
		/// <param name="aabb">Returns the bounding box for this part.</param>
		public override void BoundingBox(out AlignedBox aabb)
		{
			aabb.Minimum = new Vector3(
				MathHelper.Min(World.P1.X, World.P2.X) - World.Radius,
				MathHelper.Min(World.P1.Y, World.P2.Y) - World.Radius,
				MathHelper.Min(World.P1.Z, World.P2.Z) - World.Radius);
			aabb.Maximum = new Vector3(
				MathHelper.Max(World.P1.X, World.P2.X) + World.Radius,
				MathHelper.Max(World.P1.Y, World.P2.Y) + World.Radius,
				MathHelper.Max(World.P1.Z, World.P2.Z) + World.Radius);
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
