using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Content;

namespace Henge3D
{
	/// <summary>
	/// Represents a collection or composition of collision parts that make up the collision skin for a single object.
	/// The composition may contain one or many parts.
	/// </summary>
	public class Composition
	{
		internal uint Flags;
		internal AlignedBox BoundingBox = AlignedBox.Null;
		private List<Part> _parts;

		/// <summary>
		/// Construct a new composition.
		/// </summary>
		public Composition()
		{
			_parts = new List<Part>();
		}

		protected IList<Part> Parts { get { return _parts; } }

		/// <summary>
		/// Gets the total number of parts contained in the composition.
		/// </summary>
		public int Count { get { return _parts.Count; } }

		/// <summary>
		/// Gets a single part.
		/// </summary>
		/// <param name="i">The index of the part to get.</param>
		/// <returns>Returns a single part of the composition.</returns>
		public Part this[int i] { get { return _parts[i]; } }

		/// <summary>
		/// Add a new part to the composition.
		/// </summary>
		/// <param name="part">The part to add. This object must not already be owned by another composition.</param>
		public void Add(Part part)
		{
			if (part.Owner != null)
				throw new ArgumentException("The part already belongs to a composition.");
			part.Owner = this;
			_parts.Add(part);
			AlignedBox tmp;
			part.BoundingBox(out tmp);
			AlignedBox.Merge(ref BoundingBox, ref tmp, out BoundingBox);
		}

		/// <summary>
		/// Adds multiple parts to the composition.
		/// </summary>
		/// <param name="parts">The array containing the parts to add. The parts must not already be owned by another composition.</param>
		public void Add(params Part[] parts)
		{
			for (int i = 0; i < parts.Length; i++)
				this.Add(parts[i]);
		}

		/// <summary>
		/// Applies the specified transform to bring all parts of the composition into world-space.
		/// </summary>
		/// <param name="transform">The world-space transform to apply.</param>
		public virtual void ApplyTransform(ref Transform transform)
		{
			for (int i = 0; i < _parts.Count; i++)
			{
				_parts[i].ApplyTransform(ref transform);
			}

			if (_parts.Count > 0)
			{
				AlignedBox tmp;
				_parts[0].BoundingBox(out BoundingBox);
				for (int i = 1; i < _parts.Count; i++)
				{
					_parts[i].BoundingBox(out tmp);
					AlignedBox.Merge(ref BoundingBox, ref tmp, out BoundingBox);
				}
			}
		}

		/// <summary>
		/// Intersects a segment with all parts of the composition and returns the intersection point that is nearest to the
		/// beginning of the segment.
		/// </summary>
		/// <param name="segment">The segment to intersect with.</param>
		/// <param name="scalar">Returns a value between 0 and 1 indicating where on the segment the first intersection occurs.</param>
		/// <param name="point">Returns the point of the first intersection.</param>
		/// <returns>Returns a value indicating whether the segment intersects with any part of the composition.</returns>
		public bool Intersect(ref Segment s, out float scalar, out Vector3 point)
		{
			scalar = float.PositiveInfinity;
			point = Vector3.Zero;
			float scalar1;
			Vector3 point1;
			for (int i = 0; i < _parts.Count; i++)
			{
				if (_parts[i].Intersect(ref s, out scalar1, out point1) && scalar1 < scalar)
				{
					scalar = scalar1;
					point = point1;
				}
			}
			return !float.IsPositiveInfinity(scalar);
		}
	}
}
