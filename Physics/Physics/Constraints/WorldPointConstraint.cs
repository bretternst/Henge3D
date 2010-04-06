using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace Henge3D.Physics
{
	/// <summary>
	/// Attempts to constrain a point on a body to a point in world coordinates.
	/// </summary>
	public class WorldPointConstraint : Constraint
	{
		private Vector3 _bodyPoint, _worldOffset, _worldPoint, _impulse;
		private Matrix _mass;

		/// <summary>
		/// Construct a new world point constraint.
		/// </summary>
		/// <param name="body">The body to constrain.</param>
		/// <param name="worldPoint">The point in world coordinates to constrain the point to. The body is expected to be occupying this
		/// point at the time the constraint is created.</param>
		public WorldPointConstraint(RigidBody body, Vector3 worldPoint)
			: base(body, body)
		{
			_worldPoint = worldPoint;
			Vector3.Transform(ref worldPoint, ref BodyA.WorldInverse.Combined, out _bodyPoint);
		}

		/// <summary>
		/// Gets or sets the point in world coordinates to which the body is constrained.
		/// </summary>
		public Vector3 WorldPoint { get { return _worldPoint; } set { _worldPoint = value; } }

		/// <summary>
		/// Prepares the constraint for iterative processing in the current frame.
		/// </summary>
		public override void PreProcess()
		{
			RigidBody a = BodyA;

			// get points and mass in world coordinates
			Vector3.Transform(ref _bodyPoint, ref a.World.Combined, out _worldOffset);
			Vector3.Subtract(ref _worldOffset, ref a.World.Position, out _worldOffset);

			a.MassWorld.InverseMassMatrix(ref _worldOffset, out _mass);
			Matrix.Invert(ref _mass, out _mass);

			if (this.Manager.IsSolverWarmStarted)
			{
				Vector3 impulse;
				Vector3.Multiply(ref _impulse, this.Manager.TimeStep, out impulse);
				a.ApplyImpulse(ref impulse, ref _worldOffset);
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

			// calculate impulse required to halt motion
			Vector3 impulse;
			a.GetVelocityAtPoint(ref _worldOffset, out impulse);
			Vector3.Negate(ref impulse, out impulse);
			Vector3.Transform(ref impulse, ref _mass, out impulse);

			// add to accumulated impulse for warm starting (no clamping)
			Vector3.Multiply(ref impulse, this.Manager.TimeStepInverse, out impulse);
			Vector3.Add(ref _impulse, ref impulse, out _impulse);

			// apply impulse
			Vector3.Multiply(ref impulse, this.Manager.TimeStep, out impulse);
			a.ApplyImpulse(ref impulse, ref _worldOffset);
		}

		/// <summary>
		/// Solve the constraint for position.
		/// </summary>
		/// <returns>Returns a value indicating whether the constraint has been satisfied.</returns>
		public override bool ProcessPosition()
		{
			RigidBody a = BodyA;

			// recalculate points in world coordinates
			Vector3 impulse;
			Vector3.Transform(ref _bodyPoint, ref a.World.Combined, out _worldOffset);
			Vector3.Subtract(ref _worldPoint, ref _worldOffset, out impulse);
			Vector3.Subtract(ref _worldOffset, ref a.World.Position, out _worldOffset);

			float error = impulse.Length();
			if (error <= this.Manager.LinearErrorTolerance)
				return true;

			// need normalized direction to calculate effective mass
			Vector3 n;
			Vector3.Divide(ref impulse, error, out n);
			float mass = a.MassWorld.EffectiveMass(ref _worldOffset, ref n);
			Vector3.Multiply(ref impulse, mass * this.Manager.PositionCorrectionFactor, out impulse);

			// apply impulse
			a.ApplyFlashImpulse(ref impulse, ref _worldOffset);

			return false;
		}
	}
}
