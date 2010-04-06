using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Henge3D.Physics
{
	/// <summary>
	/// Represents a collection of constraints that together make up a single joint. A joint could contain several different constraint
	/// types to make it work as a whole.
	/// </summary>
	public class Joint : Constraint
	{
		private List<Constraint> _constraints;

		/// <summary>
		/// Construct a new joint.
		/// </summary>
		/// <param name="bodyA">The first body to constrain.</param>
		/// <param name="bodyB">The second body to constrain.</param>
		public Joint(RigidBody bodyA, RigidBody bodyB)
			: base(bodyA, bodyB)
		{
			_constraints = new List<Constraint>();
		}

		/// <summary>
		/// Gets the list of constraints included in the joint.
		/// </summary>
		public IList<Constraint> Constraints { get { return _constraints; } }

		/// <summary>
		/// Prepares the constraints for iterative processing in the current frame.
		/// </summary>
		public override void PreProcess()
		{
			for (int i = 0; i < _constraints.Count; i++)
			{
				_constraints[i].Manager = this.Manager;
				_constraints[i].PreProcess();
			}
		}

		/// <summary>
		/// Solve the constraints for velocity by applying impulses.
		/// </summary>
		public override void ProcessVelocity()
		{
			for (int i = 0; i < _constraints.Count; i++)
				_constraints[i].ProcessVelocity();
		}

		/// <summary>
		/// Solve the constraints for position.
		/// </summary>
		/// <returns>Returns a value indicating whether all constraints has been satisfied.</returns>
		public override bool ProcessPosition()
		{
			bool satisfied = true;
			for (int i = 0; i < _constraints.Count; i++)
				satisfied &= _constraints[i].ProcessPosition();
			return satisfied;
		}
	}
}
