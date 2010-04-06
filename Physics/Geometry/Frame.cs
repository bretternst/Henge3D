using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace Henge3D
{
	[Flags]
	public enum Axes
	{
		None = 0,
		X = 1,
		Y = 2,
		Z = 4,
		All = 7
	}

	/// <summary>
	/// Represents a frame, which is defined by a specified origin and three orthogonal basis directions.
	/// </summary>
	public struct Frame
	{
		public static readonly Frame Identity = new Frame(Vector3.UnitX, Vector3.UnitY, Vector3.UnitZ, Vector3.Zero);

		public Vector3 X;
		public Vector3 Y;
		public Vector3 Z;
		public Vector3 Origin;

		/// <summary>
		/// Construct a new frame. At least two orthonormal axes must be provided. If the third axes is omitted by passing in
		/// a zero vector, then it is computed from the other two.
		/// </summary>
		/// <param name="x">The X axis.</param>
		/// <param name="y">The Y axis.</param>
		/// <param name="z">The Z axis.</param>
		/// <param name="origin">The frame's origin.</param>
		public Frame(Vector3 x, Vector3 y, Vector3 z, Vector3 origin)
		{
			X = x;
			Y = y;
			Z = z;
			Origin = origin;
			this.Normalize();
		}

		/// <summary>
		/// Construct a new frame with the default basis and a specified origin.
		/// </summary>
		/// <param name="origin">The frame's origin.</param>
		public Frame(Vector3 origin)
		{
			X = Vector3.UnitX;
			Y = Vector3.UnitY;
			Z = Vector3.UnitZ;
			Origin = origin;
		}

		/// <summary>
		/// Compute any missing axes, normalize all axis vectors, and ensure that the frame is valid by checking that the
		/// axes are orthonormal.
		/// </summary>
		public void Normalize()
		{
			// right-handed coordinate system (rotate counter-clockwise)
			// <= 1 of the axes can be zero to start with
			if (X == Vector3.Zero)
				Vector3.Cross(ref Y, ref Z, out X);
			else if (Y == Vector3.Zero)
				Vector3.Cross(ref X, ref Z, out Y);
			else if (Z == Vector3.Zero)
				Vector3.Cross(ref X, ref Y, out Z);

			// normalize all vectors
			if(Math.Abs(X.LengthSquared() - 1f) >= Constants.Epsilon)
				X.Normalize();
			if(Math.Abs(Y.LengthSquared() - 1f) >= Constants.Epsilon)
				Y.Normalize();
			if(Math.Abs(Z.LengthSquared() - 1f) >= Constants.Epsilon)
				Z.Normalize();

			// validate
			if (Vector3.Dot(X, Y) >= Constants.Epsilon ||
				Vector3.Dot(X, Z) >= Constants.Epsilon ||
				Vector3.Dot(Y, Z) >= Constants.Epsilon)
			{
				throw new ArgumentException("Axes are not orthogonal.");
			}
		}

		/// <summary>
		/// Compute the Euler angles defined by this frame relative to the default frame with a zero origin and standard world axes.
		/// </summary>
		/// <param name="angles">Returns the Euler angles in radians.</param>
		public void ComputeEulerAnglesXYZ(out Vector3 angles)
		{
			angles = Vector3.Zero;
			if (X.Z - 1f < Constants.Epsilon)
			{
				if (X.Z + 1f > -Constants.Epsilon)
				{
					angles.X = (float)Math.Atan2(Y.Z, Z.Z);
					angles.Y = (float)Math.Asin(-X.Z);
					angles.Z = (float)Math.Atan2(X.Y, X.X);
				}
				else
				{
					angles.X = (float)-Math.Atan2(Y.X, Y.Y);
					angles.Y = -MathHelper.PiOver2;
					angles.Z = 0f;
				}
			}
			else
			{
				angles.X = (float)Math.Atan2(Y.X, Y.Y);
				angles.Y = MathHelper.PiOver2;
				angles.Z = 0f;
			}
		}

		/// <summary>
		/// Converts the frame to a change of basis matrix. Translation is not included, only orientation.
		/// </summary>
		/// <param name="output"></param>
		public void ToMatrix(out Matrix output)
		{
			output = new Matrix(
				X.X, X.Y, X.Z, 0f,
				Y.X, Y.Y, Y.Z, 0f,
				Z.X, Z.Y, Z.Z, 0f,
				0f, 0f, 0f, 1f);
		}

		/// <summary>
		/// Compute the difference between two frames and return the result as a new frame from the point of view of the first.
		/// </summary>
		/// <param name="b1">The base frame.</param>
		/// <param name="b2">The frame to subtract from the base frame.</param>
		/// <param name="output">Returns the difference between the two frames in the form of a third frame.</param>
		public static void Subtract(ref Frame b1, ref Frame b2, out Frame output)
		{
			// apply inverse of b2's rotation to b1
			Matrix m1, m2, rel;
			b1.ToMatrix(out m1);
			b2.ToMatrix(out m2);
			Matrix.Transpose(ref m2, out m2);
			Matrix.Multiply(ref m1, ref m2, out rel);
			output.X = new Vector3(rel.M11, rel.M12, rel.M13);
			output.Y = new Vector3(rel.M21, rel.M22, rel.M23);
			output.Z = new Vector3(rel.M31, rel.M32, rel.M33);

			// subtract b2's position from b1
			Vector3.Subtract(ref b1.Origin, ref b2.Origin, out output.Origin);
		}

		/// <summary>
		/// Apply a transform to a frame to produce a new frame.
		/// </summary>
		/// <param name="input">The frame to transform.</param>
		/// <param name="transform">The transform to apply.</param>
		/// <param name="output">Returns the new, transformed frame.</param>
		public static void Transform(ref Frame input, ref Transform transform, out Frame output)
		{
			Vector3.Transform(ref input.X, ref transform.Orientation, out output.X);
			Vector3.Transform(ref input.Y, ref transform.Orientation, out output.Y);
			Vector3.Transform(ref input.Z, ref transform.Orientation, out output.Z);
			Vector3.Transform(ref input.Origin, ref transform.Combined, out output.Origin);
		}
	}
}
