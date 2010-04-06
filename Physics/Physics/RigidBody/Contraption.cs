using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Henge3D.Physics
{
	/// <summary>
	/// Combines a number of different bodies, constraints and force generators into a single object that can be added or removed
	/// from the physics library as a single object.
	/// </summary>
	public abstract class Contraption
	{
		private List<RigidBody> _bodies;
		private List<Constraint> _constraints;
		private List<IForceGenerator> _generators;

		/// <summary>
		/// Gets the list of bodies included in the contraption.
		/// </summary>
		public IList<RigidBody> Bodies { get { return _bodies; } }

		/// <summary>
		/// Gets the list of constraints included in the contraption.
		/// </summary>
		public IList<Constraint> Constraints { get { return _constraints; } }

		/// <summary>
		/// Gets the list of generators included in the contraption.
		/// </summary>
		public IList<IForceGenerator> Generators { get { return _generators; } }

		/// <summary>
		/// Construct a new contraption.
		/// </summary>
		public Contraption()
		{
			_bodies = new List<RigidBody>();
			_constraints = new List<Constraint>();
			_generators = new List<IForceGenerator>();
		}
	}
}
