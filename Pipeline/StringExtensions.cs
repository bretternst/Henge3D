using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Content.Pipeline;

namespace Henge3D.Pipeline
{
	static class StringExtensions
	{
		public static int[] ToIntArray(this string s)
		{
			try
			{
				var values = from raw in s.Split(',')
							 let trimmed = raw.Trim()
							 where trimmed.Length > 0
							 select int.Parse(trimmed);
				return values.ToArray();
			}
			catch (FormatException ex)
			{
				throw new InvalidContentException("Invalid int array.", ex);
			}
		}

		public static float[] ToFloatArray(this string s)
		{
			try
			{
				var values = from raw in s.Split(',')
							 let trimmed = raw.Trim()
							 where trimmed.Length > 0
							 select float.Parse(trimmed);
				return values.ToArray();
			}
			catch (FormatException ex)
			{
				throw new InvalidContentException("Invalid float array.", ex);
			}
		}

		public static Vector3[] ToVector3Array(this string s)
		{
			var values = s.ToFloatArray();
			if (values.Length % 3 != 0)
				throw new InvalidContentException("Values in a vertex array specification must be a multiple of 3.");
			var output = new Vector3[values.Length / 3];
			for (int i = 0; i < values.Length; i += 3)
			{
				output[i / 3] = new Vector3(values[i], values[i + 1], values[i + 2]);
			}
			return output;
		}
	}
}
