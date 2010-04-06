using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace Henge3D.Collision
{
	/// <summary>
	/// Provides abstract method signatures for a narrow phase collision detection implementation that processes
	/// two parts for intersection and returns collision points.
	/// </summary>
	public abstract class NarrowPhase
	{
		/// <summary>
		/// When overridden in a derived class, performs a static overlap test between two parts.
		/// </summary>
		/// <param name="cf">The functor to which all collisions are reported.</param>
		/// <param name="partA">The first part to test.</param>
		/// <param name="partB">The second part to test.</param>
		public abstract void OverlapTest(CollisionFunctor cf, Part partA, Part partB);

		/// <summary>
		/// When overridden in a derived class, performs a swept (or simulated-swept) test between two parts.
		/// </summary>
		/// <param name="cf">The functor to which all collisions are reported.</param>
		/// <param name="partA">The first part to test.</param>
		/// <param name="partB">The second part to test.</param>
		/// <param name="delta">The direction and magnitude of movement of partA, relative to partB.</param>
		public abstract void SweptTest(CollisionFunctor cf, Part partA, Part partB, Vector3 delta);
	}
}
