using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace Henge3D
{
	/// <summary>
	/// Represents a transform, as defined by a change of basis, change of position, and possibly change of scale.
	/// </summary>
	public struct Transform
	{
		public float Scale;
		public Vector3 Position;
		public Quaternion Orientation;
		public Matrix Combined;

		public static Transform Identity = new Transform { Scale = 1f, Position = Vector3.Zero, Orientation = Quaternion.Identity, Combined = Matrix.Identity };

		/// <summary>
		/// Construct a new transform.
		/// </summary>
		/// <param name="scale">The change in scale multiplier.</param>
		/// <param name="position">The change in position.</param>
		/// <param name="orientation">The transform's orientation.</param>
		public Transform(float scale, Vector3 position, Quaternion orientation)
			: this (scale, ref position, ref orientation)
		{
		}

		/// <summary>
		/// Construct a new transform.
		/// </summary>
		/// <param name="scale">The change in scale multiplier.</param>
		/// <param name="position">The change in position.</param>
		/// <param name="orientation">The transform's orientation.</param>
		public Transform(float scale, ref Vector3 position, ref Quaternion orientation)
		{
			Scale = scale;
			Position = position;
			Orientation = orientation;
			Orientation.Normalize();
			Combine(Scale, ref Position, ref Orientation, out Combined);
		}

		/// <summary>
		/// Modify the transform by adding the specified delta, modified by the specified scale multiplier.
		/// </summary>
		/// <param name="scale">Scales the delta before applying it to the transform. To apply the delta exactly, use a scale of 1.</param>
		/// <param name="delta">The delta to apply to the transform.</param>
		public void ApplyDelta(float scale, ref TransformDelta delta)
		{
			Vector3 movement;

		    Vector3.Multiply(ref delta.Linear, scale, out movement);
		    Vector3.Add(ref Position, ref movement, out Position);

			Vector3 axis = delta.Angular;
			float length = axis.Length() * scale;
			if (length >= Constants.Epsilon)
			{
				axis.Normalize();
				Quaternion rotation;
				Quaternion.CreateFromAxisAngle(ref axis, length, out rotation);
				Quaternion.Multiply(ref rotation, ref Orientation, out Orientation);
				Orientation.Normalize();
			}

			Combine(Scale, ref Position, ref Orientation, out Combined);
		}

		/// <summary>
		/// Compute the inverse of the transform. When the inverse of the transform is applied, it undoes the effects of the transform.
		/// </summary>
		/// <param name="inverted">Returns the inverted transform.</param>
		public void Invert(out Transform inverted)
		{
			inverted.Scale = 1f / Scale;
			Quaternion.Inverse(ref Orientation, out inverted.Orientation);
			Vector3.Negate(ref Position, out inverted.Position);
			CombineInverse(inverted.Scale, ref inverted.Position, ref inverted.Orientation, out inverted.Combined);
		}

		private static void Combine(float scale, ref Vector3 position, ref Quaternion orientation,
			out Matrix output)
		{
			Matrix.CreateScale(scale, out output);
			Matrix rotation;
			Matrix.CreateFromQuaternion(ref orientation, out rotation);
			Matrix.Multiply(ref output, ref rotation, out output);
			output.M41 = position.X;
			output.M42 = position.Y;
			output.M43 = position.Z;
		}

		private static void CombineInverse(float scaleInv, ref Vector3 positionInv, ref Quaternion orientationInv,
			out Matrix worldInverse)
		{
			Matrix tmp;

			Matrix rotationInv;
			Matrix.CreateFromQuaternion(ref orientationInv, out rotationInv);

			Matrix.CreateTranslation(ref positionInv, out worldInverse);
			Matrix.Multiply(ref worldInverse, ref rotationInv, out worldInverse);
			Matrix.CreateScale(scaleInv, out tmp);
			Matrix.Multiply(ref worldInverse, ref tmp, out worldInverse);
		}
	}
}
