using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace Henge3D.Physics
{
	/// <summary>
	/// Constrains a body to a specified world position which can change over time. This constraint is useful, for example, for
	/// dragging objects around with a mouse. It is less strict than WorldPointConstraint, preferring to use velocity impulses only to solve
	/// the constraint.
	/// </summary>
	public class GrabConstraint : Constraint
	{
		private Vector3 _bodyPoint, _worldPoint, _worldOffset, _normal, _impulse;
		private float _maxForce = float.PositiveInfinity, _factor = 20f;
		private Matrix _mass;

		/// <summary>
		/// Construct a new grab constraint.
		/// </summary>
		/// <param name="body">The body to constrain.</param>
		/// <param name="worldPoint">The initial world point to which the body should be constrained.</param>
		/// <param name="bodyPoint">The initial body point, in world coordinates, that the constraint is applied to.</param>
		public GrabConstraint(RigidBody body, Vector3 worldPoint, Vector3 bodyPoint)
			: base(body, body)
		{
			_worldPoint = worldPoint;
			_bodyPoint = bodyPoint;
		}

		/// <summary>
		/// Construct a new grab constraint.
		/// </summary>
		/// <param name="body">The body to constrain.</param>
		/// <param name="worldPoint">The initial world point to which the body should be constrained. It is expected that the body
		/// is occupying this point when the constraint is created.</param>
		public GrabConstraint(RigidBody body, Vector3 worldPoint)
			: this(body, worldPoint, Vector3.Transform(worldPoint, body.WorldInverse.Combined))
		{
		}

		/// <summary>
		/// Gets or sets the world point to which the body should be constrained.
		/// </summary>
		public Vector3 WorldPoint { get { return _worldPoint; } set { _worldPoint = value; } }

		/// <summary>
		/// Gets or sets the maximum force that can be applied to solve the constraint.
		/// </summary>
		public float MaxForce { get { return _maxForce; } set { _maxForce = value; } }

		/// <summary>
		/// Gets or sets the multiplier, controlling how much of the distance error the constraint attempts to solve each frame.
		/// Solving only part of the distance error can make dragging behavior smoother.
		/// </summary>
		public float Factor { get { return _factor; } set { _factor = value; } }

		/// <summary>
		/// Prepares the constraint for iterative processing in the current frame.
		/// </summary>
		public override void PreProcess()
		{
			RigidBody a = this.BodyA;

			Vector3.Transform(ref _bodyPoint, ref a.World.Combined, out _worldOffset);
			Vector3.Subtract(ref _worldPoint, ref _worldOffset, out _normal);
			Vector3.Subtract(ref _worldOffset, ref a.World.Position, out _worldOffset);
			Vector3.Multiply(ref _normal, _factor, out _normal);

			a.MassWorld.InverseMassMatrix(ref _worldOffset, out _mass);
			Matrix.Invert(ref _mass, out _mass);

			if(this.Manager.IsSolverWarmStarted)
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
			RigidBody a = this.BodyA;

			Vector3 impulse;
			a.GetVelocityAtPoint(ref _worldOffset, out impulse);
			Vector3.Negate(ref impulse, out impulse);
			Vector3.Add(ref impulse, ref _normal, out impulse);
			Vector3.Transform(ref impulse, ref _mass, out impulse);

			// clamp the accumulated impulse against the max force
			Vector3.Multiply(ref impulse, this.Manager.TimeStepInverse, out impulse);
			Vector3 oldImpulse = _impulse;
			Vector3.Add(ref _impulse, ref impulse, out _impulse);
			if (_impulse.LengthSquared() > _maxForce * _maxForce)
			{
				Vector3.Multiply(ref _impulse, _maxForce / _impulse.Length(), out _impulse);
			}
			Vector3.Subtract(ref _impulse, ref oldImpulse, out impulse);
			Vector3.Multiply(ref impulse, this.Manager.TimeStep, out impulse);

			a.ApplyImpulse(ref impulse, ref _worldOffset);
		}
	}
}
