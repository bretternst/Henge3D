using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework.Content.Pipeline;

namespace Henge3D.Pipeline
{
	public static class OpaqueDataDictionaryExtensions
	{
		public static T GetAttribute<T>(this OpaqueDataDictionary data, string key, T defaultValue)
		{
			if (!data.ContainsKey(key)) return defaultValue;
			object o = data[key];

			if (typeof(T) == o.GetType())
			{
				return (T)o;
			}
			else if (typeof(T).IsEnum && o is string)
			{
				string val = (string)o;
				try
				{
					return (T)Enum.Parse(typeof(T), val, true);
				}
				catch (ArgumentException)
				{
					throw new PipelineException(string.Format("Unrecognized enum value '{0}' for property '{1}.", val, key));
				}
			}

			return defaultValue;
		}
	}
}
