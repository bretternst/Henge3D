using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace Henge3D.Physics
{
	/// <summary>
	/// Attempts to constrain the relative position and orientation of two bodies relative to each other. This constraint provides
	/// the ability to limit each of six degrees of freedom (translation along three axes, and rotation around three axes).
	/// </summary>
	public class GenericConstraint : Constraint
	{
		private enum LimitState
		{
			Inactive,
			Locked,
			Min,
			Max
		}

		private Frame _bodyBasisA, _bodyBasisB, _worldBasisA, _worldBasisB;
		private LimitState _stateRotX, _stateRotY, _stateRotZ, _statePosX, _statePosY, _statePosZ;
		private Axes _limitedRotAxes, _limitedPosAxes;
		private Vector3 _minAngles, _maxAngles, _minPos, _maxPos, _axisX, _axisY, _axisZ, _angles, _positions, _anchorA, _anchorB,
			_impulseRot, _impulsePos, _inertia, _mass;
		private Vector3 _pImpulsePosX, _pImpulsePosY, _pImpulsePosZ, _pImpulseRotX, _pImpulseRotY, _pImpulseRotZ;

		/// <summary>
		/// Construct a new generic constraint.
		/// </summary>
		/// <param name="bodyA">The first constrained body.</param>
		/// <param name="bodyB">The second constrained body.</param>
		/// <param name="basis">The world-space frame of the second body against which the first body will be constrained. This value
		/// must contain the initial position and orientation of the second body at the time the constraint was created.</param>
		/// <param name="limitedPosAxes">Specifies which axes around which to constrain position.</param>
		/// <param name="minPos">The minimum distances between the two bodies along each axis.</param>
		/// <param name="maxPos">The maximum distances between the two bodies along each axis.</param>
		/// <param name="limitedRotAxes">Specifies which axes around which to constrain rotation.</param>
		/// <param name="minAngles">The minimum relative angles, in radians, around each axis.</param>
		/// <param name="maxAngles">The maximum relative angles, in radians, around each axis.</param>
		public GenericConstraint(RigidBody bodyA, RigidBody bodyB, Frame basis, Axes limitedPosAxes,
			Vector3 minPos, Vector3 maxPos, Axes limitedRotAxes, Vector3 minAngles, Vector3 maxAngles)
			: base(bodyA, bodyB)
		{
			Frame.Transform(ref basis, ref bodyA.WorldInverse, out _bodyBasisA);
			Frame.Transform(ref basis, ref bodyB.WorldInverse, out _bodyBasisB);
			_limitedPosAxes = limitedPosAxes;
			_limitedRotAxes = limitedRotAxes;
			_minPos = minPos;
			_maxPos = maxPos;
			_minAngles = minAngles;
			_maxAngles = maxAngles;
		}

		/// <summary>
		/// Prepares the constraint for iterative processing in the current frame.
		/// </summary>
		public override void PreProcess()
		{
			ComputeVitals();

			LimitState prevX = _statePosX, prevY = _statePosY, prevZ = _statePosZ;

			_statePosX = ComputeLinearLimitState((_limitedPosAxes & Axes.X) > 0, _positions.X, _minPos.X, _maxPos.X);
			_statePosY = ComputeLinearLimitState((_limitedPosAxes & Axes.Y) > 0, _positions.Y, _minPos.Y, _maxPos.Y);
			_statePosZ = ComputeLinearLimitState((_limitedPosAxes & Axes.Z) > 0, _positions.Z, _minPos.Z, _maxPos.Z);
			_stateRotX = ComputeAngularLimitState((_limitedRotAxes & Axes.X) > 0, _angles.X, _minAngles.X, _maxAngles.X);
			_stateRotY = ComputeAngularLimitState((_limitedRotAxes & Axes.Y) > 0, _angles.Y, _minAngles.Y, _maxAngles.Y);
			_stateRotZ = ComputeAngularLimitState((_limitedRotAxes & Axes.Z) > 0, _angles.Z, _minAngles.Z, _maxAngles.Z);

			DoLinearWarmStart(prevX, _statePosX, ref _impulsePos.X, ref _worldBasisB.X);
			DoLinearWarmStart(prevY, _statePosY, ref _impulsePos.Y, ref _worldBasisB.Y);
			DoLinearWarmStart(prevZ, _statePosZ, ref _impulsePos.Z, ref _worldBasisB.Z);

			_impulseRot = Vector3.Zero;
			_pImpulsePosX = _pImpulsePosY = _pImpulsePosZ = Vector3.Zero;
			_pImpulseRotX = _pImpulseRotY = _pImpulseRotZ = Vector3.Zero;
		}

		/// <summary>
		/// Solve the constraint for velocity by applying impulses.
		/// </summary>
		public override void ProcessVelocity()
		{
			RigidBody a = this.BodyA, b = this.BodyB;

			// solve linear constraints
			Vector3 va, vb;
			a.GetVelocityAtPoint(ref _anchorA, out va);
			b.GetVelocityAtPoint(ref _anchorB, out vb);
			Vector3.Subtract(ref va, ref vb, out va);
			this.SolveLinearVelocity(_statePosX, ref va, ref _worldBasisB.X, ref _impulsePos.X, _mass.X);
			this.SolveLinearVelocity(_statePosY, ref va, ref _worldBasisB.Y, ref _impulsePos.Y, _mass.Y);
			this.SolveLinearVelocity(_statePosZ, ref va, ref _worldBasisB.Z, ref _impulsePos.Z, _mass.Z);

			// solve angular constraints
			Vector3.Subtract(ref a.Velocity.Angular, ref b.Velocity.Angular, out va);
			this.SolveAngularVelocity(_stateRotX, ref va, ref _axisX, ref _impulseRot.X, _inertia.X);
			this.SolveAngularVelocity(_stateRotY, ref va, ref _axisY, ref _impulseRot.Y, _inertia.Y);
			this.SolveAngularVelocity(_stateRotZ, ref va, ref _axisZ, ref _impulseRot.Z, _inertia.Z);
		}

		/// <summary>
		/// Solve the constraint for position.
		/// </summary>
		/// <returns>Returns a value indicating whether the constraint has been satisfied.</returns>
		public override bool ProcessPosition()
		{
			RigidBody a = BodyA, b = BodyB;

			ComputeVitals();

			bool satisfied = true;
			satisfied &= this.SolveLinearPosition(_statePosX, _positions.X, _minPos.X, _maxPos.X, ref _worldBasisB.X, ref _pImpulsePosX, _mass.X);
			satisfied &= this.SolveLinearPosition(_statePosY, _positions.Y, _minPos.Y, _maxPos.Y, ref _worldBasisB.Y, ref _pImpulsePosY, _mass.Y);
			satisfied &= this.SolveLinearPosition(_statePosZ, _positions.Z, _minPos.Z, _maxPos.Z, ref _worldBasisB.Z, ref _pImpulsePosZ, _mass.Z);

			satisfied &= this.SolveAngularPosition(_stateRotX, _angles.X, _minAngles.X, _maxAngles.X, ref _axisX, ref _pImpulseRotX, _inertia.X);
			satisfied &= this.SolveAngularPosition(_stateRotY, _angles.Y, _minAngles.Y, _maxAngles.Y, ref _axisY, ref _pImpulseRotY, _inertia.Y);
			satisfied &= this.SolveAngularPosition(_stateRotZ, _angles.Z, _minAngles.Z, _maxAngles.Z, ref _axisZ, ref _pImpulseRotZ, _inertia.Z);

			return satisfied;
		}

		private void SolveLinearVelocity(LimitState state, ref Vector3 vel, ref Vector3 axis, ref float accImpulse, float mass)
		{
			RigidBody a = this.BodyA, b = this.BodyB;

			if (state == LimitState.Inactive)
				return;

			float impulse, oldImpulse = accImpulse;
			Vector3.Dot(ref axis, ref vel, out impulse);
			impulse *= -mass;

			// clamp the impulse
			accImpulse += impulse;
			if (state == LimitState.Min && accImpulse < 0f || state == LimitState.Max && accImpulse > 0f)
				accImpulse = 0f;
			impulse = accImpulse - oldImpulse;

			Vector3 i;
			Vector3.Multiply(ref axis, impulse, out i);
			a.ApplyImpulse(ref i, ref _anchorA);
			Vector3.Negate(ref i, out i);
			b.ApplyImpulse(ref i, ref _anchorB);
		}

		private void DoLinearWarmStart(LimitState prevState, LimitState state, ref float accImpulse, ref Vector3 axis)
		{
			RigidBody a = this.BodyA, b = this.BodyB;

			if (prevState == state)
			{
				Vector3 i;
				Vector3.Multiply(ref axis, accImpulse, out i);
				a.ApplyImpulse(ref i, ref _anchorA);
				Vector3.Negate(ref i, out i);
				b.ApplyImpulse(ref i, ref _anchorB);
			}
			else
				accImpulse = 0f;
		}
		
		private void SolveAngularVelocity(LimitState state, ref Vector3 vel, ref Vector3 axis, ref float accImpulse, float mass)
		{
			RigidBody a = this.BodyA, b = this.BodyB;

			if (state == LimitState.Inactive)
				return;

			float impulse, oldImpulse = accImpulse;
			Vector3.Dot(ref axis, ref vel, out impulse);
			impulse *= -mass;

			// clamp the impulse
			accImpulse += impulse;
			if (state == LimitState.Min && accImpulse < 0f || state == LimitState.Max && accImpulse > 0f)
				accImpulse = 0f;
			impulse = accImpulse - oldImpulse;

			Vector3 i;
			Vector3.Multiply(ref axis, impulse, out i);
			a.ApplyAngularImpulse(ref i);
			Vector3.Negate(ref i, out i);
			b.ApplyAngularImpulse(ref i);
		}

		private bool SolveLinearPosition(LimitState state, float pos, float minPos, float maxPos, ref Vector3 axis, ref Vector3 accImpulse, float mass)
		{
			RigidBody a = this.BodyA, b = this.BodyB;

			float error = 0f;
			switch (state)
			{
				case LimitState.Inactive:
					return true;
				case LimitState.Locked:
					if (pos < minPos)
						error = minPos - pos;
					else if (pos > maxPos)
						error = maxPos - pos;
					else
						error = 0f;
					break;
				case LimitState.Min:
					error = minPos - pos;
					break;
				case LimitState.Max:
					error = maxPos - pos;
					break;
			}
			if (Math.Abs(error) <= this.Manager.LinearErrorTolerance)
				return true;

			Vector3 impulse, oldImpulse = accImpulse;
			Vector3.Multiply(ref axis, error * mass * this.Manager.PositionCorrectionFactor, out impulse);
			Vector3.Add(ref accImpulse, ref impulse, out accImpulse);
			float d;
			Vector3.Dot(ref axis, ref accImpulse, out d);
			if (state == LimitState.Min && d < 0f || state == LimitState.Max && d > 0f)
				accImpulse = Vector3.Zero;
			Vector3.Subtract(ref accImpulse, ref oldImpulse, out impulse);

			a.ApplyFlashImpulse(ref impulse, ref _anchorA);
			Vector3.Negate(ref impulse, out impulse);
			b.ApplyFlashImpulse(ref impulse, ref _anchorB);

			return false;
		}

		private bool SolveAngularPosition(LimitState state, float angle, float minAngle, float maxAngle, ref Vector3 axis, ref Vector3 accImpulse, float mass)
		{
			RigidBody a = this.BodyA, b = this.BodyB;

			float error = 0f;
			switch (state)
			{
				case LimitState.Inactive:
					return true;
				case LimitState.Locked:
					if (angle < minAngle)
						error = minAngle - angle;
					else if (angle > maxAngle)
						error = maxAngle - angle;
					else
						error = 0f;
					break;
				case LimitState.Min:
					error = minAngle - angle;
					break;
				case LimitState.Max:
					error = maxAngle - angle;
					break;
			}
			if (Math.Abs(error) < this.Manager.AngularErrorTolerance)
				return true;

			Vector3 impulse, oldImpulse = accImpulse;
			Vector3.Multiply(ref axis, error * mass * this.Manager.PositionCorrectionFactor, out impulse);
			Vector3.Add(ref accImpulse, ref impulse, out accImpulse);
			float d;
			Vector3.Dot(ref axis, ref accImpulse, out d);
			if (state == LimitState.Min && d < 0f || state == LimitState.Max && d > 0f)
				accImpulse = Vector3.Zero;
			Vector3.Subtract(ref accImpulse, ref oldImpulse, out impulse);

			a.ApplyAngularFlashImpulse(ref impulse);
			Vector3.Negate(ref impulse, out impulse);
			b.ApplyAngularFlashImpulse(ref impulse);

			return false;
		}

		private void ComputeVitals()
		{
			RigidBody a = this.BodyA, b = this.BodyB;

			// compute relative basis between the two objects in world position
			Frame.Transform(ref _bodyBasisA, ref this.BodyA.World, out _worldBasisA);
			Frame.Transform(ref _bodyBasisB, ref this.BodyB.World, out _worldBasisB);
			Frame rel;
			Frame.Subtract(ref _worldBasisA, ref _worldBasisB, out rel);

			// compute current positions and angles
			Vector3.Subtract(ref _worldBasisA.Origin, ref _worldBasisB.Origin, out _positions);
			Matrix m;
			_worldBasisB.ToMatrix(out m);
			Matrix.Transpose(ref m, out m);
			Vector3.Transform(ref _positions, ref m, out _positions);
			rel.ComputeEulerAnglesXYZ(out _angles);

			// borrowed from Bullet; construct the axes about which we actually restrict rotation
			Vector3.Cross(ref _worldBasisB.Z, ref _worldBasisA.X, out _axisY);
			Vector3.Cross(ref _axisY, ref _worldBasisB.Z, out _axisX);
			Vector3.Cross(ref _worldBasisA.X, ref _axisY, out _axisZ);
			_axisX.Normalize();
			_axisY.Normalize();
			_axisZ.Normalize();

			// calculate effective inertia along each axis
			Matrix inertia = new Matrix();
			if (a.IsMovable)
				Matrix.Add(ref inertia, ref a.MassWorld.InertiaInverse, out inertia);
			if (b.IsMovable)
				Matrix.Add(ref inertia, ref b.MassWorld.InertiaInverse, out inertia);
			inertia.M44 = 1f;
			Matrix.Invert(ref inertia, out inertia);
			Vector3 d;
			Vector3.Transform(ref _axisX, ref inertia, out d);
			Vector3.Dot(ref d, ref _axisX, out _inertia.X);
			Vector3.Transform(ref _axisY, ref inertia, out d);
			Vector3.Dot(ref d, ref _axisY, out _inertia.Y);
			Vector3.Transform(ref _axisZ, ref inertia, out d);
			Vector3.Dot(ref d, ref _axisZ, out _inertia.Z);

			// calculate effective mass along each axis of basis B
			//float w = b.MassWorld.MassInverse / (a.MassWorld.MassInverse = b.MassWorld.MassInverse);
			//Vector3.Multiply(ref _worldBasisA.Origin, 1f - w, out _anchorA);
			//Vector3.Multiply(ref _worldBasisB.Origin, w, out _anchorB);
			//Vector3.Add(ref _anchorA, ref _anchorB, out _anchorB);
			//Vector3.Subtract(ref _anchorB, ref a.World.Position, out _anchorA);
			//Vector3.Subtract(ref _anchorB, ref b.World.Position, out _anchorB);
			Vector3.Subtract(ref _worldBasisA.Origin, ref a.World.Position, out _anchorA);
			Vector3.Subtract(ref _worldBasisB.Origin, ref b.World.Position, out _anchorB);
			_mass.X = MassProperties.EffectiveMass(ref a.MassWorld, ref b.MassWorld, ref _anchorA, ref _anchorB, ref _worldBasisB.X);
			_mass.Y = MassProperties.EffectiveMass(ref a.MassWorld, ref b.MassWorld, ref _anchorA, ref _anchorB, ref _worldBasisB.Y);
			_mass.Z = MassProperties.EffectiveMass(ref a.MassWorld, ref b.MassWorld, ref _anchorA, ref _anchorB, ref _worldBasisB.Z);
		}

		private LimitState ComputeLinearLimitState(bool isActive, float pos, float minPos, float maxPos)
		{
			if (!isActive)
				return LimitState.Inactive;

			if (Math.Abs(maxPos - minPos) < this.Manager.LinearErrorTolerance * 2f)
				return LimitState.Locked;
			else if (pos <= minPos)
				return LimitState.Min;
			else if (pos >= maxPos)
				return LimitState.Max;
			else
				return LimitState.Inactive;
		}

		private LimitState ComputeAngularLimitState(bool isActive, float angle, float minAngle, float maxAngle)
		{
			if(!isActive)
				return LimitState.Inactive;

			if (Math.Abs(maxAngle - minAngle) < this.Manager.AngularErrorTolerance * 2f)
				return LimitState.Locked;
			else if (angle <= minAngle)
				return LimitState.Min;
			else if (angle >= maxAngle)
				return LimitState.Max;
			else
				return LimitState.Inactive;
		}
	}
}
