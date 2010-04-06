using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace Henge3D.Collision
{
	/// <summary>
	/// Provides abstract method signatures for a broad phase collision detection implementation.
	/// </summary>
	public abstract class BroadPhase
	{
		/// <summary>
		/// When overridden in a derived class, adds a new composition to be considered by the implementation.
		/// </summary>
		/// <param name="c">The composition to consider.</param>
		public abstract void Add(Composition c);

		/// <summary>
		/// When overridden in a derived class, removes a composition from consideration.
		/// </summary>
		/// <param name="c">The composition to remove.</param>
		public abstract void Remove(Composition c);

		/// <summary>
		/// When overridden in a derived class, clears all compositions from the implementation.
		/// </summary>
		public abstract void Clear();

		/// <summary>
		/// When overridden in a derived class, executes the broad phase collision detection and provides
		/// results to the functor.
		/// </summary>
		/// <param name="cf">The functor that will receive the results of the collision detection.</param>
		public abstract void Execute(CollisionFunctor cf);

		/// <summary>
		/// When overridden in a derived class, intersects a segment with all compositions and returns the
		/// point of intersection nearest to the beginning of the segment.
		/// </summary>
		/// <param name="segment">The line segment to intersect with.</param>
		/// <param name="scalar">Returns a value between 0 and 1 indicating where in the segment the intersection occurred.</param>
		/// <param name="point">Returns the point of the intersection nearest to the beginning of the segment.</param>
		/// <returns></returns>
		public abstract Composition Intersect(ref Segment segment, out float scalar, out Vector3 point);
	}
}
