﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Content;

namespace Henge3D.Physics
{
	/// <summary>
	/// Represents the collision skin of a rigid body, composed of one or more collision parts.
	/// </summary>
	public sealed class BodySkin : Composition
	{
		private Dictionary<Part, Material> _materials;
		private RigidBody _owner;
		private Material _material;
		private Transform _transformInverse;

		internal BodySkin(RigidBody owner)
		{
			_owner = owner;
			_materials = new Dictionary<Part, Material>();
		}

		/// <summary>
		/// Gets a reference to the body to which this skin belongs.
		/// </summary>
		public RigidBody Owner { get { return _owner; } }

		/// <summary>
		/// Gets or sets the material to be applied by default to all parts.
		/// </summary>
		public Material DefaultMaterial { get { return _material; } set { _material = value; } }

		/// <summary>
		/// Add a new part to the collision skin.
		/// </summary>
		/// <param name="part">The part to add.</param>
		/// <param name="material">The material to apply to the part.</param>
		public void Add(Part part, Material material)
		{
			base.Add(part);
			_materials.Add(part, material);
		}

		/// <summary>
		/// Gets the material from the specified part.
		/// </summary>
		/// <param name="part">The part for which to retrieve the material.</param>
		/// <returns>Returns the part's material.</returns>
		public Material Material(Part part)
		{
			return _materials.ContainsKey(part) ? _materials[part] : _material;
		}

		/// <summary>
		/// Apply a transform to all parts in the collision skin to bring them into world coordinates.
		/// </summary>
		/// <param name="transform">The transform to apply.</param>
		public override void ApplyTransform(ref Transform transform)
		{
			base.ApplyTransform(ref transform);

			transform.Invert(out _transformInverse);
		}

		internal void ApplySweep(ref Vector3 delta)
		{
			BoundingBox.Sweep(ref delta);
		}

		internal void UndoTransform(ref Vector3 p)
		{
			Vector3.Transform(ref p, ref _transformInverse.Combined, out p);
			Vector3.Transform(ref p, ref _owner.World.Combined, out p);
		}
	}
}