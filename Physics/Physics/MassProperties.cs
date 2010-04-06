using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace Henge3D.Physics
{
	/// <summary>
	/// Maintains mass properties for a specific body, including the body's total mass and inertia.
	/// </summary>
	public struct MassProperties
	{
		public float Mass;
		public float MassInverse;
		public Matrix Inertia;
		public Matrix InertiaInverse;

		/// <summary>
		/// A set of mass properties with infinite mass and inertia, rendering a body immobile.
		/// </summary>
		public static MassProperties Immovable = new MassProperties(
			float.PositiveInfinity, Matrix.Identity * float.PositiveInfinity);

		/// <summary>
		/// Construct a new set of mass properties.
		/// </summary>
		/// <param name="mass">The body's total mass.</param>
		/// <param name="inertia">The body's inertia with respect to its center of mass.</param>
		public MassProperties(float mass, Matrix inertia)
		{
			Mass = mass;
			MassInverse = 1f / mass;
			Inertia = inertia;
			Matrix.Invert(ref Inertia, out InertiaInverse);
		}

		internal void InverseMassMatrix(ref Vector3 worldOffset, out Matrix output)
		{
			var c = worldOffset;
			var i = InertiaInverse;
			output = new Matrix(
				c.Z * c.Z * i.M22 - c.Y * c.Z * (i.M23 + i.M32) + c.Y * c.Y * i.M33 + MassInverse,
				-(c.Z * c.Z * i.M21) + c.X * c.Z * i.M23 + c.Y * c.Z * i.M31 - c.X * c.Y * i.M33,
				c.Y * c.Z * i.M21 - c.X * c.Z * i.M22 - c.Y * c.Y * i.M31 + c.X * c.Y * i.M32,
				0f,
				-(c.Z * c.Z * i.M12) + c.Y * c.Z * i.M13 + c.X * c.Z * i.M32 - c.X * c.Y * i.M33,
				c.Z * c.Z * i.M11 - c.X * c.Z * (i.M13 + i.M31) + c.X * c.X * i.M33 + MassInverse,
				-(c.Y * c.Z * i.M11) + c.X * c.Z * i.M12 + c.X * c.Y * i.M31 - c.X * c.X * i.M32,
				0f,
				c.Y * c.Z * i.M12 - c.Y * c.Y * i.M13 - c.X * c.Z * i.M22 + c.X * c.Y * i.M23,
				-(c.Y * c.Z * i.M11) + c.X * c.Y * i.M13 + c.X * c.Z * i.M21 - c.X * c.X * i.M23,
				c.Y * c.Y * i.M11 - c.X * c.Y * (i.M12 + i.M21) + c.X * c.X * i.M22 + MassInverse,
				0f, 0f, 0f, 0f, 1f
				);
		}

		internal float EffectiveMass(ref Vector3 offset, ref Vector3 n)
		{
			Vector3 v;
			float normalMass = 0f, f;
			if (Mass < float.PositiveInfinity)
			{
				normalMass = MassInverse;
				Vector3.Cross(ref offset, ref n, out v);
				Vector3.Transform(ref v, ref InertiaInverse, out v);
				Vector3.Cross(ref v, ref offset, out v);
				Vector3.Dot(ref n, ref v, out f);
				normalMass += f;
			}
			if (normalMass < Constants.Epsilon)
				normalMass = Constants.Epsilon;
			return 1f / normalMass;
		}

		const float div6 = 1.0f / 6.0f;
		const float div24 = 1.0f / 24.0f;
		const float div60 = 1.0f / 60.0f;
		const float div120 = 1.0f / 120.0f;

		/// <summary>
		/// Computes mass properties from a triangle mesh with a given density.
		/// </summary>
		/// <param name="density">The density of the material.</param>
		/// <param name="vertices">An array of all vertices in the triangle.</param>
		/// <param name="indices">An array of indices, defining the triangles in the mesh. The vertices must be specified
		/// in counter-clockwise order in groups of three.</param>
		/// <param name="centerOfMass">Returns the computed center of mass.</param>
		/// <returns>Returns the calculated mass properties.</returns>
		public static MassProperties FromTriMesh(float density, Vector3[] vertices, int[] indices, out Vector3 centerOfMass)
		{
			float scale = 1f;
			float in0 = 0f, in1 = 0f, in2 = 0f, in3 = 0f, in4 = 0f, in5 = 0f, in6 = 0f, in7 = 0f, in8 = 0f, in9 = 0f;
			int numFaces = indices.Length / 3;
			for (int i = 0; i < numFaces; i++)
			{
				int i0 = i * 3, i1 = i * 3 + 1, i2 = i * 3 + 2;
				float x0 = vertices[indices[i0]].X * scale, y0 = vertices[indices[i0]].Y * scale, z0 = vertices[indices[i0]].Z * scale;
				float x1 = vertices[indices[i1]].X * scale, y1 = vertices[indices[i1]].Y * scale, z1 = vertices[indices[i1]].Z * scale;
				float x2 = vertices[indices[i2]].X * scale, y2 = vertices[indices[i2]].Y * scale, z2 = vertices[indices[i2]].Z * scale;

				float a1 = x1 - x0, b1 = y1 - y0, c1 = z1 - z0;
				float a2 = x2 - x0, b2 = y2 - y0, c2 = z2 - z0;
				float d0 = b1 * c2 - b2 * c1;
				float d1 = a2 * c1 - a1 * c2;
				float d2 = a1 * b2 - a2 * b1;

				// compute integral terms
				float temp0, temp1, temp2;

				temp0 = x0 + x1;
				temp1 = x0 * x0;
				temp2 = temp1 + x1 * temp0;
				float f1x = temp0 + x2;
				float f2x = temp2 + x2 * f1x;
				float f3x = x0 * temp1 + x1 * temp2 + x2 * f2x;
				float g0x = f2x + x0 * (f1x + x0);
				float g1x = f2x + x1 * (f1x + x1);
				float g2x = f2x + x2 * (f1x + x2);

				temp0 = y0 + y1;
				temp1 = y0 * y0;
				temp2 = temp1 + y1 * temp0;
				float f1y = temp0 + y2;
				float f2y = temp2 + y2 * f1y;
				float f3y = y0 * temp1 + y1 * temp2 + y2 * f2y;
				float g0y = f2y + y0 * (f1y + y0);
				float g1y = f2y + y1 * (f1y + y1);
				float g2y = f2y + y2 * (f1y + y2);

				temp0 = z0 + z1;
				temp1 = z0 * z0;
				temp2 = temp1 + z1 * temp0;
				float f1z = temp0 + z2;
				float f2z = temp2 + z2 * f1z;
				float f3z = z0 * temp1 + z1 * temp2 + z2 * f2z;
				float g0z = f2z + z0 * (f1z + z0);
				float g1z = f2z + z1 * (f1z + z1);
				float g2z = f2z + z2 * (f1z + z2);

				in0 += d0 * f1x;
				in1 += d0 * f2x;
				in2 += d1 * f2y;
				in3 += d2 * f2z;
				in4 += d0 * f3x;
				in5 += d1 * f3y;
				in6 += d2 * f3z;
				in7 += d0 * (y0 * g0x + y1 * g1x + y2 * g2x);
				in8 += d1 * (z0 * g0y + z1 * g1y + z2 * g2y);
				in9 += d2 * (x0 * g0z + x1 * g1z + x2 * g2z);
			}

			in0 *= div6; in1 *= div24; in2 *= div24; in3 *= div24; in4 *= div60; in5 *= div60; in6 *= div60;
			in7 *= div120; in8 *= div120; in9 *= div120;

			centerOfMass = new Vector3(in1 / in0, in2 / in0, in3 / in0);
			var inertia = new Matrix(
				in5 + in6, -in7, -in9, 0f,
				-in7, in4 + in6, -in8, 0f,
				-in9, -in8, in4 + in5, 0f,
				0f, 0f, 0f, 1f);
			inertia.M11 -= in0 * (centerOfMass.Y * centerOfMass.Y + centerOfMass.Z * centerOfMass.Z);
			inertia.M22 -= in0 * (centerOfMass.Z * centerOfMass.Z + centerOfMass.X * centerOfMass.X);
			inertia.M33 -= in0 * (centerOfMass.X * centerOfMass.X + centerOfMass.Y * centerOfMass.Y);
			inertia.M12 += in0 * centerOfMass.X * centerOfMass.Y;
			inertia.M21 += in0 * centerOfMass.X * centerOfMass.Y;
			inertia.M23 += in0 * centerOfMass.Y * centerOfMass.Z;
			inertia.M32 += in0 * centerOfMass.Y * centerOfMass.Z;
			inertia.M13 += in0 * centerOfMass.Z * centerOfMass.X;
			inertia.M31 += in0 * centerOfMass.Z * centerOfMass.X;
			Matrix.Multiply(ref inertia, density, out inertia);

			TranslateInertiaTensor(ref inertia, in0 * density, centerOfMass, out inertia);

			return new MassProperties(in0 * density, inertia);
		}

		/// <summary>
		/// Computes the mass properties for a sphere.
		/// </summary>
		/// <param name="density">The material density.</param>
		/// <param name="center">The center of the sphere.</param>
		/// <param name="radius">The radius of the sphere.</param>
		/// <returns>Returns the calculated mass properties.</returns>
		public static MassProperties FromSphere(float density, Vector3 center, float radius)
		{
			float m = (4f / 3f) * MathHelper.Pi * (float)Math.Pow(radius, 3.0f) * density;
			float i = (2f / 5f) * m * radius * radius;

			var tensor = new Matrix(
					i, 0f, 0f, 0f,
					0f, i, 0f, 0f,
					0f, 0f, i, 0f,
					0f, 0f, 0f, 1f);
			TranslateInertiaTensor(ref tensor, m, center, out tensor);
			return new MassProperties(m, tensor);
		}

		/// <summary>
		/// Computes the mass properties for a capsule.
		/// </summary>
		/// <param name="density">The material density.</param>
		/// <param name="p1">The first endpoint of the segment that defines the capsule.</param>
		/// <param name="p2">The second endpoint of the segment that defines the capsule.</param>
		/// <param name="radius">The capsule's radius.</param>
		/// <param name="center">Returns the computed center of mass.</param>
		/// <returns>Returns the calculated mass properties.</returns>
		public static MassProperties FromCapsule(float density, Vector3 p1, Vector3 p2, float radius, out Vector3 center)
		{
			float length;
			Vector3 d;
			Vector3.Distance(ref p1, ref p2, out length);
			Vector3.Subtract(ref p2, ref p1, out d);
			Vector3.Divide(ref d, length, out d);
			Vector3.Multiply(ref d, 0.5f * length, out center);
			Vector3.Add(ref p1, ref center, out center);

			Matrix orientation;
			d = Vector3.Cross(Vector3.UnitZ, d);
			float rotate = d.Length();
			if (rotate < Constants.Epsilon)
				orientation = Matrix.Identity;
			else
			{
				Vector3.Divide(ref d, rotate, out d);
				Matrix.CreateFromAxisAngle(ref d, rotate, out orientation);
			}

			float mCyl = density * MathHelper.Pi * radius * radius * length;
			float mEnd = density * (2f / 3f) * MathHelper.Pi * radius * radius * radius;
			float ixxyy = (1f/12f) * mCyl * (3 * radius * radius + length * length) +
				+ (2f/5f) * mEnd * radius * radius + mEnd * (0.5f * length) * (0.5f * length);
			float izz = (1f/2f) * mCyl * radius * radius + (2f / 5f) * mEnd * radius * radius;
			var i = new Matrix(
				ixxyy, 0f, 0f, 0f,
				0f, ixxyy, 0f, 0f,
				0f, 0f, izz, 0f,
				0f, 0f, 0f, 1f);
			Matrix inv;
			Matrix.Transpose(ref orientation, out inv);
			Matrix.Multiply(ref orientation, ref i, out i);
			Matrix.Multiply(ref i, ref inv, out i);
			i.M44 = 0f;

			float m = mCyl + 2 * mEnd;

			TranslateInertiaTensor(ref i, m, center, out i);
			return new MassProperties(m, i);
		}

		/// <summary>
		/// Computes the mass properties of an axis-aligned cuboid shape (cube or other box).
		/// </summary>
		/// <param name="density">The material density.</param>
		/// <param name="dimensions">The dimensions of the cube along each axis.</param>
		/// <returns></returns>
		public static MassProperties FromCuboid(float density, Vector3 dimensions)
		{
			float m = dimensions.X * dimensions.Y * dimensions.Z;
			float f = (1f / 12f) * m;
			var i = new Matrix(
				f * (dimensions.Y * dimensions.Y + dimensions.Z * dimensions.Z), 0f, 0f, 0f,
				0f, f * (dimensions.X * dimensions.X + dimensions.Z * dimensions.Z), 0f, 0f,
				0f, 0f, f * (dimensions.X * dimensions.X + dimensions.Y * dimensions.Y), 0f,
				0f, 0f, 0f, 1f);

			return new MassProperties(m, i);
		}

		/// <summary>
		/// Computes a new inertia tensor matrix from a given matrix by applying a translation (shift in position).
		/// </summary>
		/// <param name="tensor">The source inertia tensor.</param>
		/// <param name="mass">The body's total mass.</param>
		/// <param name="offset">The direction and magnitude to shift the translate tensor.</param>
		/// <param name="output">Returns the translated inertia tensor.</param>
		public static void TranslateInertiaTensor(ref Matrix tensor, float mass, Vector3 offset, out Matrix output)
		{
			float m = mass, x = offset.X, y = offset.Y, z = offset.Z;
			var t = new Matrix(
				m * (y * y + z * z), -m * x * y, -m * x * z, 0f,
				-m * y * x, m * (z * z + x * x), -m * y * z, 0f,
				-m * z * x, -m * z * y, m * (x * x + y * y), 0f,
				0f, 0f, 0f, 1f);
			Matrix.Add(ref tensor, ref t, out output);
		}

		internal static void Transform(ref MassProperties mass, ref Transform transform, out MassProperties output)
		{
			output.Mass = mass.Mass;
			output.MassInverse = mass.MassInverse;
			Matrix orientation, orientationInv;
			Matrix.CreateFromQuaternion(ref transform.Orientation, out orientation);
			Matrix.Transpose(ref orientation, out orientationInv);
			Matrix.Multiply(ref orientationInv, ref mass.Inertia, out output.Inertia);
			Matrix.Multiply(ref output.Inertia, ref orientation, out output.Inertia);
			Matrix.Multiply(ref orientationInv, ref mass.InertiaInverse, out output.InertiaInverse);
			Matrix.Multiply(ref output.InertiaInverse, ref orientation, out output.InertiaInverse);
		}

		internal static float EffectiveMass(ref MassProperties a, ref MassProperties b, ref Vector3 offsetA, ref Vector3 offsetB, ref Vector3 normal)
		{
			Vector3 v;
			float normalMass = 0f, f;
			if (a.Mass < float.PositiveInfinity)
			{
				normalMass = a.MassInverse;
				Vector3.Cross(ref offsetA, ref normal, out v);
				Vector3.Transform(ref v, ref a.InertiaInverse, out v);
				Vector3.Cross(ref v, ref offsetA, out v);
				Vector3.Dot(ref normal, ref v, out f);
				normalMass += f;
			}
			if (b.Mass < float.PositiveInfinity)
			{
				normalMass += b.MassInverse;
				Vector3.Cross(ref offsetB, ref normal, out v);
				Vector3.Transform(ref v, ref b.InertiaInverse, out v);
				Vector3.Cross(ref v, ref offsetB, out v);
				Vector3.Dot(ref normal, ref v, out f);
				normalMass += f;
			}
			if (normalMass < Constants.Epsilon)
				normalMass = Constants.Epsilon;
			return 1f / normalMass;
		}

		internal static void EffectiveMassMatrix(ref MassProperties a, ref MassProperties b, ref Vector3 offsetA, ref Vector3 offsetB, out Matrix k)
		{
			Matrix ka = new Matrix(), kb = new Matrix();
			if(a.Mass < float.PositiveInfinity) a.InverseMassMatrix(ref offsetA, out ka);
			if(b.Mass < float.PositiveInfinity) b.InverseMassMatrix(ref offsetB, out kb);
			Matrix.Add(ref ka, ref kb, out k);
			k.M44 = 1f;
			Matrix.Invert(ref k, out k);
		}
	}
}
