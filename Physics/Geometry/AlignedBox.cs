using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace Henge3D
{
	public enum BoxIntersectType
	{
		None,
		Overlap,
		AContainsB,
		BContainsA
	}

	/// <summary>
	/// Represents an axis-aligned bounding box represented by minimum and maximum values along each world axis.
	/// </summary>
	public struct AlignedBox
	{
		public static readonly AlignedBox Infinite = new AlignedBox(
			new Vector3(float.NegativeInfinity, float.NegativeInfinity, float.NegativeInfinity),
			new Vector3(float.PositiveInfinity, float.PositiveInfinity, float.PositiveInfinity));
		public static readonly AlignedBox Null = new AlignedBox(
			new Vector3(float.PositiveInfinity, float.PositiveInfinity, float.PositiveInfinity),
			new Vector3(float.NegativeInfinity, float.NegativeInfinity, float.NegativeInfinity)
			);

		public Vector3 Minimum;
		public Vector3 Maximum;

		/// <summary>
		/// Construct a new aligned bounding box.
		/// </summary>
		/// <param name="min">The minimum X, Y and Z values.</param>
		/// <param name="max">The maximum X, Y and Z values.</param>
		public AlignedBox(Vector3 min, Vector3 max)
		{
			Minimum = min;
			Maximum = max;
		}

		/// <summary>
		/// Stretches the bounding box to include the specified point, if it does not already.
		/// </summary>
		/// <param name="p">The new point to include.</param>
		public void Add(ref Vector3 p)
		{
			Vector3.Min(ref Minimum, ref p, out Minimum);
			Vector3.Max(ref Maximum, ref p, out Maximum);
		}

		/// <summary>
		/// Sweep the bounding box along the specified direction, extending its boundaries by a certain magnitude and in a certain direction.
		/// </summary>
		/// <param name="delta">The magnitude and direction to extend the bounding box.</param>
		public void Sweep(ref Vector3 delta)
		{
			float f;
			Vector3 d;

			d = Vector3.UnitX;
			Vector3.Dot(ref d, ref delta, out f);
			if (f >= 0f) Maximum.X += f;
			else Minimum.X += f;

			d = Vector3.UnitY;
			Vector3.Dot(ref d, ref delta, out f);
			if (f >= 0f) Maximum.Y += f;
			else Minimum.Y += f;

			d = Vector3.UnitZ;
			Vector3.Dot(ref d, ref delta, out f);
			if (f >= 0f) Maximum.Z += f;
			else Minimum.Z += f;
		}

		/// <summary>
		/// Construct a new bounding box that encapsulates a cloud of points.
		/// </summary>
		/// <param name="vertices">The list of vertices to enclose in the bounding box.</param>
		/// <param name="aabb">Returns the bounding box containing all points.</param>
		public static void Fit(IList<Vector3> vertices, out AlignedBox aabb)
		{
			aabb = AlignedBox.Null;
			for (int i = 0; i < vertices.Count; i++)
			{
				var v = vertices[i];
				Vector3.Min(ref v, ref aabb.Minimum, out aabb.Minimum);
				Vector3.Max(ref v, ref aabb.Maximum, out aabb.Maximum);
			}
		}

		/// <summary>
		/// Constructs a new bounding box that encapsulates a series of points.
		/// </summary>
		/// <param name="p1">The first point.</param>
		/// <param name="p2">The second point.</param>
		/// <param name="aabb">Returns the bounding box containing all points.</param>
		public static void Fit(ref Vector3 p1, ref Vector3 p2, out AlignedBox aabb)
		{
			aabb = new AlignedBox(p1, p1);
			aabb.Add(ref p2);
		}

		/// <summary>
		/// Constructs a new bounding box that encapsulates a series of points.
		/// </summary>
		/// <param name="p1">The first point.</param>
		/// <param name="p2">The second point.</param>
		/// <param name="p3">The third point.</param>
		/// <param name="aabb">Returns the bounding box containing all points.</param>
		public static void Fit(ref Vector3 p1, ref Vector3 p2, ref Vector3 p3, out AlignedBox aabb)
		{
			aabb = new AlignedBox(p1, p1);
			aabb.Add(ref p2);
			aabb.Add(ref p3);
		}

		/// <summary>
		/// Combine two bounding boxes to produce a new bounding box that fully contains both.
		/// </summary>
		/// <param name="aabb1">The first bounding box.</param>
		/// <param name="aabb2">The second bounding box.</param>
		/// <param name="output">Returns the bounding box fully containing both inputs.</param>
		public static void Merge(ref AlignedBox aabb1, ref AlignedBox aabb2, out AlignedBox output)
		{
			Vector3.Min(ref aabb1.Minimum, ref aabb2.Minimum, out output.Minimum);
			Vector3.Max(ref aabb1.Maximum, ref aabb2.Maximum, out output.Maximum);
		}

		/// <summary>
		/// Intersect two bounding boxes.
		/// </summary>
		/// <param name="a">The first bounding box.</param>
		/// <param name="b">The second bonding box.</param>
		/// <returns>Returns a value indicating the type of intersection.</returns>
		public static BoxIntersectType Intersect(ref AlignedBox a, ref AlignedBox b)
		{
			if (b.Minimum.X <= a.Maximum.X && b.Maximum.X >= a.Minimum.X &&
				b.Minimum.Y <= a.Maximum.Y && b.Maximum.Y >= a.Minimum.Y &&
				b.Minimum.Z <= a.Maximum.Z && b.Maximum.Z >= a.Minimum.Z)
			{
				if (b.Minimum.X >= a.Minimum.X && b.Maximum.X <= a.Maximum.X &&
					b.Minimum.Y >= a.Minimum.Y && b.Maximum.Y <= a.Maximum.Y &&
					b.Minimum.Z >= a.Minimum.Z && b.Maximum.Z <= a.Maximum.Z)
					return BoxIntersectType.AContainsB;
				else if (a.Minimum.X >= b.Minimum.X && a.Maximum.X <= b.Maximum.X &&
					a.Minimum.Y >= b.Minimum.Y && a.Maximum.Y <= b.Maximum.Y &&
					a.Minimum.Z >= b.Minimum.Z && a.Maximum.Z <= b.Maximum.Z)
					return BoxIntersectType.BContainsA;
				else
					return BoxIntersectType.Overlap;
			}
			return BoxIntersectType.None;
		}

		/// <summary>
		/// Transforms a bounding box into world-space or local-space using the specified transform. The box may grow
		/// significantly depending on the size and new/old basis axes.
		/// </summary>
		/// <param name="box">The bounding box to transform.</param>
		/// <param name="transform">The transformation to apply.</param>
		/// <param name="output">Returns the transformed bounding box.</param>
		public static void Transform(ref AlignedBox box, ref Transform transform, out AlignedBox output)
		{
			Vector3 dim = box.Maximum - box.Minimum;
			Vector3 p1, p2, p3, p4, p5, p6, p7, p8;
			p1 = p2 = p3 = p4 = p5 = p6 = p7 = p8 = box.Minimum;
			p2.X += dim.X;
			p3.X += dim.X; p3.Y += dim.Y;
			p4.Y += dim.Y;
			p5.Z += dim.Z;
			p6.X += dim.X; p6.Z += dim.Z;
			p7.X += dim.X; p7.Y += dim.Y; p7.Z += dim.Z;
			p8.Y += dim.Y; p8.Z += dim.Z;
			Vector3.Transform(ref p1, ref transform.Combined, out p1);
			Vector3.Transform(ref p2, ref transform.Combined, out p2);
			Vector3.Transform(ref p3, ref transform.Combined, out p3);
			Vector3.Transform(ref p4, ref transform.Combined, out p4);
			Vector3.Transform(ref p5, ref transform.Combined, out p5);
			Vector3.Transform(ref p6, ref transform.Combined, out p6);
			Vector3.Transform(ref p7, ref transform.Combined, out p7);
			Vector3.Transform(ref p8, ref transform.Combined, out p8);
			output = AlignedBox.Null;
			output.Add(ref p1);
			output.Add(ref p2);
			output.Add(ref p3);
			output.Add(ref p4);
			output.Add(ref p5);
			output.Add(ref p6);
			output.Add(ref p7);
			output.Add(ref p8);
		}
	}
}
