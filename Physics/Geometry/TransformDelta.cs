using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace Henge3D
{
	/// <summary>
	/// Represents a change to a transform, defined as an addition to the translation and orientation components of the transform.
	/// </summary>
	public struct TransformDelta
	{
		public Vector3 Linear;
		public Vector3 Angular;

		/// <summary>
		/// Construct a new delta.
		/// </summary>
		/// <param name="linear">The linear change.</param>
		/// <param name="angular">The angular change, specified in axis-magnitude form.</param>
		public TransformDelta(ref Vector3 linear, ref Vector3 angular)
		{
			Linear = linear;
			Angular = angular;
		}

		/// <summary>
		/// Construct a new delta.
		/// </summary>
		/// <param name="linear">The linear change.</param>
		/// <param name="axis">The normalized axis around which rotation is applied.</param>
		/// <param name="angular">The magnitude of rotation, in radians.</param>
		public TransformDelta(ref Vector3 linear, ref Vector3 axis, float angular)
		{
			Linear = linear;
			Vector3.Multiply(ref axis, angular, out Angular);
		}

		/// <summary>
		/// Gets a value indicating whether the delta will have an effect when applied to a transform.
		/// </summary>
		public bool HasValue
		{
			get
			{
				return Linear.LengthSquared() >= Constants.Epsilon ||
					Angular.LengthSquared() >= Constants.Epsilon;
			}
		}

		/// <summary>
		/// Combine the specified changes in position and orientation with the delta.
		/// </summary>
		/// <param name="linearDelta">The additional position change.</param>
		/// <param name="angularDelta">The additional rotation.</param>
		public void Add(ref Vector3 linearDelta, ref Vector3 angularDelta)
		{
			Vector3.Add(ref Linear, ref linearDelta, out Linear);
			Vector3.Add(ref Angular, ref angularDelta, out Angular);
		}

		/// <summary>
		/// Scale the delta by the specified multiplier.
		/// </summary>
		/// <param name="factor">The scaling multiplier.</param>
		public void Scale(float factor)
		{
			Vector3.Multiply(ref Linear, factor, out Linear);
			Vector3.Multiply(ref Angular, factor, out Angular);
		}

		/// <summary>
		/// Set the delta to zero so that it has no effect when applied to a transform.
		/// </summary>
		public void SetToZero()
		{
			Linear = Vector3.Zero;
			Angular = Vector3.Zero;
		}

		/// <summary>
		/// Clamp the delta against the specified limits.
		/// </summary>
		/// <param name="maxLinear">The maximum linear change.</param>
		/// <param name="maxAngular">The maximum angular change, in radians.</param>
		public void Clamp(float maxLinear, float maxAngular)
		{
			float len = Linear.LengthSquared();
			if (len > maxLinear * maxLinear)
			{
				len = Linear.Length();
				Vector3.Multiply(ref Linear, maxLinear / len, out Linear);
			}
			else if (float.IsNaN(len))
				Linear = Vector3.Zero;

			len = Angular.LengthSquared();
			if (len > maxAngular * maxAngular)
			{
				len = Angular.Length();
				Vector3.Multiply(ref Angular, maxAngular / len, out Angular);
			}
			else if (float.IsNaN(len))
				Angular = Vector3.Zero;
		}
	}
}
