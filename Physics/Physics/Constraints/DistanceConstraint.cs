using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace Henge3D.Physics
{
	/// <summary>
	/// Attempts to keep two bodies at a specified distance range from each other.
	/// </summary>
	public class DistanceConstraint : Constraint
	{
		private enum LimitState
		{
			Between,
			Equal,
			Min,
			Max
		}

		private Vector3 _bodyPointA, _bodyPointB, _worldOffsetA, _worldOffsetB, _normal, _impulse, _pImpulse;
		private float _distance, _minDistance, _maxDistance;
		private float _mass;
		private LimitState _state;

		/// <summary>
		/// Construct a new distance constraint.
		/// </summary>
		/// <param name="bodyA">The first body.</param>
		/// <param name="bodyB">The second body.</param>
		/// <param name="worldPointA">The world-space point at which the constraint is applied on the first body.</param>
		/// <param name="worldPointB">The world-space point at which the constraint is applied on the second body.</param>
		/// <param name="minDistance">The minimum allowed distance.</param>
		/// <param name="maxDistance">The maximum allowed distance.</param>
		public DistanceConstraint(RigidBody bodyA, RigidBody bodyB, Vector3 worldPointA, Vector3 worldPointB, float minDistance, float maxDistance)
			: base(bodyA, bodyB)
		{
			// store body points
			Vector3.Transform(ref worldPointA, ref bodyA.WorldInverse.Combined, out _bodyPointA);
			Vector3.Transform(ref worldPointB, ref bodyB.WorldInverse.Combined, out _bodyPointB);
			_minDistance = minDistance;
			_maxDistance = maxDistance;
		}

		/// <summary>
		/// Gets or sets the minimum allowed distance.
		/// </summary>
		public float MinDistance { get { return _minDistance; } set { _minDistance = value; } }

		/// <summary>
		/// Gets or sets the maximum allowed distance.
		/// </summary>
		public float MaxDistance { get { return _maxDistance; } set { _maxDistance = value; } }

		/// <summary>
		/// Prepares the constraint for iterative processing in the current frame.
		/// </summary>
		public override void PreProcess()
		{
			RigidBody a = BodyA, b = BodyB;

			// get world points and normal
			Vector3.Transform(ref _bodyPointA, ref a.World.Combined, out _worldOffsetA);
			Vector3.Transform(ref _bodyPointB, ref b.World.Combined, out _worldOffsetB);
			Vector3.Subtract(ref _worldOffsetA, ref _worldOffsetB, out _normal);
			Vector3.Subtract(ref _worldOffsetA, ref a.World.Position, out _worldOffsetA);
			Vector3.Subtract(ref _worldOffsetB, ref b.World.Position, out _worldOffsetB);

			_distance = _normal.Length();
			if (_distance < Constants.Epsilon)
				_normal = Vector3.Zero;
			else
				Vector3.Divide(ref _normal, _distance, out _normal);

			_mass = MassProperties.EffectiveMass(ref a.MassWorld, ref b.MassWorld, ref _worldOffsetA, ref _worldOffsetB, ref _normal);

			// determine the constraint behavior for this frame
			var prevState = _state;
			if (Math.Abs(_maxDistance - _minDistance) < 2f * this.Manager.LinearErrorTolerance)
				_state = LimitState.Equal;
			else if (_distance <= _minDistance)
				_state = LimitState.Min;
			else if (_distance >= _maxDistance)
				_state = LimitState.Max;
			else
				_state = LimitState.Between;

			if (this.Manager.IsSolverWarmStarted && prevState == _state)
			{
				Vector3 impulse;
				Vector3.Multiply(ref _impulse, this.Manager.TimeStep, out impulse);
				b.ApplyImpulse(ref impulse, ref _worldOffsetB);
				Vector3.Negate(ref impulse, out impulse);
				a.ApplyImpulse(ref impulse, ref _worldOffsetA);
			}
			else
				_impulse = Vector3.Zero;

			_pImpulse = Vector3.Zero;
		}

		/// <summary>
		/// Solve the constraint for velocity by applying impulses.
		/// </summary>
		public override void ProcessVelocity()
		{
			RigidBody a = BodyA, b = BodyB;

			if (_state == LimitState.Between)
				return;

			// calculate impulse required to zero velocity along normal
			float impulseMag;
			Vector3 va, vb, impulse;
			a.GetVelocityAtPoint(ref _worldOffsetA, out va);
			b.GetVelocityAtPoint(ref _worldOffsetB, out vb);
			Vector3.Subtract(ref va, ref vb, out va);
			Vector3.Dot(ref va, ref _normal, out impulseMag);
			Vector3.Multiply(ref _normal, impulseMag * _mass * this.Manager.TimeStepInverse, out impulse);

			// clamp the impulse appropriately for the current state
			float d;
			Vector3 oldImpulse = _impulse;
			Vector3.Add(ref _impulse, ref impulse, out _impulse);
			Vector3.Dot(ref _impulse, ref _normal, out d);
			if (_state == LimitState.Min && d > 0f || _state == LimitState.Max && d < 0f)
				_impulse = Vector3.Zero;
			Vector3.Subtract(ref _impulse, ref oldImpulse, out impulse);

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

			if (_state == LimitState.Between)
				return true;

			// recalculate vitals
			Vector3.Transform(ref _bodyPointA, ref a.World.Combined, out _worldOffsetA);
			Vector3.Transform(ref _bodyPointB, ref b.World.Combined, out _worldOffsetB);
			Vector3.Subtract(ref _worldOffsetA, ref _worldOffsetB, out _normal);
			Vector3.Subtract(ref _worldOffsetA, ref a.World.Position, out _worldOffsetA);
			Vector3.Subtract(ref _worldOffsetB, ref b.World.Position, out _worldOffsetB);
			_distance = _normal.Length();
			Vector3.Divide(ref _normal, _distance, out _normal);
			_mass = MassProperties.EffectiveMass(ref a.MassWorld, ref b.MassWorld, ref _worldOffsetA, ref _worldOffsetB, ref _normal);

			// the error depends on the current limit state
			float error = 0f;
			switch (_state)
			{
				case LimitState.Equal:
					if (_distance > _maxDistance)
						error = _distance - _maxDistance;
					else if (_distance < _minDistance)
						error = _distance - _minDistance;
					break;
				case LimitState.Min:
					error = MathHelper.Min(_distance - _minDistance, 0f);
					break;
				case LimitState.Max:
					error = MathHelper.Max(_distance - _maxDistance, 0f);
					break;
			}
			if (Math.Abs(error) <= this.Manager.LinearErrorTolerance)
				return true;

			// clamp impulse
			Vector3 impulse, oldImpulse = _pImpulse;
			Vector3.Multiply(ref _normal, error * _mass * this.Manager.PositionCorrectionFactor, out impulse);
			Vector3.Add(ref _pImpulse, ref impulse, out _pImpulse);
			float d;
			Vector3.Dot(ref _normal, ref _pImpulse, out d);
			if (_state == LimitState.Min && d > 0f || _state == LimitState.Max && d < 0f)
				_pImpulse = Vector3.Zero;
			Vector3.Subtract(ref _pImpulse, ref oldImpulse, out impulse);

			// apply impulse
			b.ApplyFlashImpulse(ref impulse, ref _worldOffsetB);
			Vector3.Negate(ref impulse, out impulse);
			a.ApplyFlashImpulse(ref impulse, ref _worldOffsetA);

			return false;
		}
	}
}
