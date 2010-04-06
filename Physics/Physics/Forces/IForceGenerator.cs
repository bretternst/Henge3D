using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace Henge3D.Physics
{
	/// <summary>
	/// Provides an interface for generating a force on a list of bodies.
	/// </summary>
	public interface IForceGenerator
	{
		/// <summary>
		/// Generate and apply a force to the bodies.
		/// </summary>
		/// <param name="bodies">The list of bodies on which to apply force.</param>
		void Generate(IList<RigidBody> bodies);
	}
}
