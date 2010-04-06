using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Content;

namespace Henge3D
{
	/// <summary>
	/// Provides base properties and abstract method definitions for all collision parts.
	/// </summary>
	public abstract class Part
	{
		private Composition _owner;

		/// <summary>
		/// Returns the composition that owns this collision part.
		/// </summary>
		public Composition Owner { get { return _owner; } internal set { _owner = value; } }

		/// <summary>
		/// When overridden in a derived class, applies a world transform to the collision part.
		/// </summary>
		/// <param name="transform">The transform used to bring the part into world-space.</param>
		public abstract void ApplyTransform(ref Transform transform);

		/// <summary>
		/// When overridden in a derived class, returns the bounding box that encapsulates the collision part in world-space.
		/// </summary>
		/// <param name="aabb">Returns the world-space bounding box.</param>
		public abstract void BoundingBox(out AlignedBox aabb);

		/// <summary>
		/// When overridden in a derived class, intersects a segment with the collision part, returning the intersection nearest
		/// to the beginning of the segment.
		/// </summary>
		/// <param name="segment">The segment to intersect with.</param>
		/// <param name="scalar">Returns a value between 0 and 1 indicating where on the segment the first intersection occurs.</param>
		/// <param name="point">Returns the point of the first intersection.</param>
		/// <returns>Returns a value indicating whether the segment intersects with the part.</returns>
		public abstract bool Intersect(ref Segment segment, out float scalar, out Vector3 point);
	}
}
