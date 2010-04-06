using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace Henge3D.Physics
{
	/// <summary>
	/// Attempts to constrain the position of a point on one body to a point on another body.
	/// </summary>
	public class PointConstraint : Constraint
	{
		private Vector3 _bodyPointA, _bodyPointB, _worldOffsetA, _worldOffsetB, _impulse;
		private Matrix _mass;

		/// <summary>
		/// Construct a new point constraint.
		/// </summary>
		/// <param name="bodyA">The first constrained body.</param>
		/// <param name="bodyB">The second constrained body.</param>
		/// <param name="worldPoint">The point in world coordinates shared by both bodies at the time the constraint is created.</param>
		public PointConstraint(RigidBody bodyA, RigidBody bodyB, Vector3 worldPoint)
			: base(bodyA, bodyB)
		{
			Vector3.Transform(ref worldPoint, ref BodyA.WorldInverse.Combined, out _bodyPointA);
			Vector3.Transform(ref worldPoint, ref BodyB.WorldInverse.Combined, out _bodyPointB);
		}

		/// <summary>
		/// Prepares the constraint for iterative processing in the current frame.
		/// </summary>
		public override void PreProcess()
		{
			RigidBody a = BodyA, b = BodyB;

			// get offsets and mass in world coordinates
			Vector3.Transform(ref _bodyPointA, ref a.World.Combined, out _worldOffsetA);
			Vector3.Subtract(ref _worldOffsetA, ref a.World.Position, out _worldOffsetA);
			Vector3.Transform(ref _bodyPointB, ref b.World.Combined, out _worldOffsetB);
			Vector3.Subtract(ref _worldOffsetB, ref b.World.Position, out _worldOffsetB);

			MassProperties.EffectiveMassMatrix(ref a.MassWorld, ref b.MassWorld, ref _worldOffsetA, ref _worldOffsetB, out _mass);

			if (this.Manager.IsSolverWarmStarted)
			{
				Vector3 impulse;
				Vector3.Multiply(ref _impulse, this.Manager.TimeStep, out impulse);
				b.ApplyImpulse(ref impulse, ref _worldOffsetB);
				Vector3.Negate(ref impulse, out impulse);
				a.ApplyImpulse(ref impulse, ref _worldOffsetA);
			}
			else
				_impulse = Vector3.Zero;
		}

		/// <summary>
		/// Solve the constraint for velocity by applying impulses.
		/// </summary>
		public override void ProcessVelocity()
		{
			RigidBody a = BodyA, b = BodyB;

			// calculate impulse required to halt motion at point
			Vector3 va, vb, impulse;
			a.GetVelocityAtPoint(ref _worldOffsetA, out va);
			b.GetVelocityAtPoint(ref _worldOffsetB, out vb);
			Vector3.Subtract(ref va, ref vb, out impulse);
			Vector3.Transform(ref impulse, ref _mass, out impulse);

			// save accumulated impulse for warm starting (no clamping)
			Vector3.Multiply(ref impulse, this.Manager.TimeStepInverse, out impulse);
			Vector3.Add(ref _impulse, ref impulse, out _impulse);

			// apply impulse
			Vector3.Multiply(ref impulse, this.Manager.TimeStep, out impulse);
			b.ApplyImpulse(ref impulse, ref _worldOffsetB);
			Vector3.Negate(ref impulse, out impulse);
			a.ApplyImpulse(ref impulse, ref _worldOffsetA);
		}

		/// <summary>
		/// Solve the constraint for position.
		/// </summary>
		/// <returns>Returns a value indicating whether the constraint has been satisfied.</returns>
		public override bool ProcessPosition()
		{
			RigidBody a = BodyA, b = BodyB;

			// get offsets and distance in world coordinates
			Vector3 impulse;
			Vector3.Transform(ref _bodyPointA, ref a.World.Combined, out _worldOffsetA);
			Vector3.Transform(ref _bodyPointB, ref b.World.Combined, out _worldOffsetB);
			Vector3.Subtract(ref _worldOffsetA, ref _worldOffsetB, out impulse);
			Vector3.Subtract(ref _worldOffsetA, ref a.World.Position, out _worldOffsetA);
			Vector3.Subtract(ref _worldOffsetB, ref b.World.Position, out _worldOffsetB);

			float error = impulse.Length();
			if (error <= this.Manager.LinearErrorTolerance)
				return true;

			// need normalized direction to calculate effective mass
			Vector3 n;
			Vector3.Divide(ref impulse, error, out n);
			float mass = MassProperties.EffectiveMass(ref a.MassWorld, ref b.MassWorld, ref _worldOffsetA, ref _worldOffsetB, ref n);
			Vector3.Multiply(ref impulse, mass * this.Manager.PositionCorrectionFactor, out impulse);

			// apply impulse
			b.ApplyFlashImpulse(ref impulse, ref _worldOffsetB);
			Vector3.Negate(ref impulse, out impulse);
			a.ApplyFlashImpulse(ref impulse, ref _worldOffsetA);

			return false;
		}
	}
}
