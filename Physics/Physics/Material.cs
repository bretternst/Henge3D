using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Henge3D.Physics
{
	/// <summary>
	/// Represents the properties of a material that can be applied to a collision part.
	/// </summary>
	public struct Material
	{
		/// <summary>
		/// The elasticity or amount of "bounce". A value of zero indicates an inelastic material (no bounce will occur).
		/// A value of 1 represents perfect elasticity.
		/// </summary>
		public float Elasticity;

		/// <summary>
		/// The roughness of the material, affecting how much friction will be applied when two bodies are in contact.
		/// A value of zero represents a completely frictionless object.
		/// </summary>
		public float Roughness;

		/// <summary>
		/// Construct a new material.
		/// </summary>
		/// <param name="elasticity">The elasticity factor.</param>
		/// <param name="roughness">The roughness factor.</param>
		public Material(float elasticity, float roughness)
		{
			Elasticity = elasticity;
			Roughness = roughness;
		}
	}
}
