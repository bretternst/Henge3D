using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace Henge3D
{
	/// <summary>
	/// Supplies helper methods for floats that are not already provided by MathHelper.
	/// </summary>
	public static class FloatHelper
	{
		/// <summary>
		/// Gets the absolute value of a number.
		/// </summary>
		/// <param name="n">A number.</param>
		/// <returns>Returns the absolute value of the nubmer.</returns>
		public static float Abs(float n)
		{
			return n < 0f ? -n : n;
		}

		/// <summary>
		/// Attempts to compare two floats.
		/// </summary>
		/// <param name="a">The first number.</param>
		/// <param name="b">The second number.</param>
		/// <returns>Returns zero if the floats are equal, a positive value if the first float is greater, and a negative value
		/// if the second number is greater.</returns>
		public static int Compare(float a, float b)
		{
			if (a == b) return 0;
			float diff = FloatHelper.Abs(a - b);
			if (diff < Constants.Epsilon) return 0;
			if (b != 0 && diff / FloatHelper.Abs(b) < Constants.Epsilon) return 0;
			return a > b ? 1 : -1;
		}

		/// <summary>
		/// Determines whether two floats are equal, or within a certain narrow range of each other.
		/// </summary>
		/// <param name="a">The first float.</param>
		/// <param name="b">The second float.</param>
		/// <returns>Returns a value indicating whether the numbers are nearly equal.</returns>
		public static bool Equals(float a, float b)
		{
			return FloatHelper.Compare(a, b) == 0;
		}
	}
}
