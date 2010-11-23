using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;

namespace Henge3D.Physics
{
	/// <summary>
	/// Provides base properties, method implementations, and abstract method signatures for classes that implement physics constraints.
	/// </summary>
	public abstract class Constraint
	{
		private static int CurrentConstraintId = 0;

		private PhysicsManager _manager;
		private Island _island;
		private RigidBody _bodyA, _bodyB;
		private bool _isCollisionEnabled = true;

		internal int ConstraintId;

		/// <summary>
		/// Construct a new constraint.
		/// </summary>
		/// <param name="bodyA">The first constrained body.</param>
		/// <param name="bodyB">The second constrained body.</param>
		public Constraint(RigidBody bodyA, RigidBody bodyB)
		{
			_bodyA = bodyA;
			_bodyB = bodyB;

			this.ConstraintId = Interlocked.Increment(ref CurrentConstraintId);
		}

		/// <summary>
		/// Gets the first constrained body.
		/// </summary>
		public RigidBody BodyA { get { return _bodyA; } protected set { _bodyA = value; } }

		/// <summary>
		/// Gets the second constrained body.
		/// </summary>
		public RigidBody BodyB { get { return _bodyB; } protected set { _bodyB = value; } }

		/// <summary>
		/// Gets a reference to the physics manager that invokes the constraint.
		/// </summary>
		public PhysicsManager Manager { get { return _manager; } internal set { _manager = value; } }

		/// <summary>
		/// Gets or sets a value that determines whether collisions are detected between the two constrained bodies.
		/// </summary>
		public bool IsCollisionEnabled { get { return _isCollisionEnabled; } set { _isCollisionEnabled = value; } }

		/// <summary>
		/// Gets the island to which the constrained bodies belong.
		/// </summary>
		internal Island Island { get { return _island; } set { _island = value; } }

		/// <summary>
		/// When overridden in a derived class, performs actions necessary to initialize the constraint for the current frame.
		/// This method is called once per frame before the constraints are solved.
		/// </summary>
		public virtual void PreProcess() { }

		/// <summary>
		/// When overridden in a derived class, solves the velocity portion of the constraint, modifying the bodies' velocities
		/// if necessary. This method can be called many times per constraint per frame.
		/// </summary>
		public virtual void ProcessVelocity() { }

		/// <summary>
		/// When overridden in a derived class, solves the position portion of the constraint, modifying the bodies' positions
		/// if necessary. This method can be called many times per constraint per frame.
		/// </summary>
		/// <returns></returns>
		public virtual bool ProcessPosition() { return true; }

		/// <summary>
		/// Reset the constraint to default values.
		/// </summary>
		public virtual void Recycle()
		{
			_island = null;
			_bodyA = _bodyB = null;
		}
	}
}
