using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace Henge3D
{
	/// <summary>
	/// Represents a line that intersects with a given point and proceeds infinitely in the given direction and the negative of
	/// the given direction.
	/// </summary>
	public struct Line
	{
		public Vector3 P;
		public Vector3 Direction;

		/// <summary>
		/// Construct a line.
		/// </summary>
		/// <param name="p">A point intersecting the line.</param>
		/// <param name="direction">The direction of the line.</param>
		public Line(Vector3 p, Vector3 direction)
		{
			P = p;
			Direction = direction;
		}
	}
}
