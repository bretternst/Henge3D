using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace Henge3D.Physics
{
	/// <summary>
	/// Attempts to constrain two bodies so that they can only rotate relative to each other along two axes. Rotation along the third
	/// axis is not permitted. The relative positions are also constrained.
	/// </summary>
	public class UniversalJoint : Joint
	{
		/// <summary>
		/// Construct a new universal joint.
		/// </summary>
		/// <param name="bodyA">The first body to constrain.</param>
		/// <param name="bodyB">The second body to constrain.</param>
		/// <param name="worldPoint">The point in world coordinates shared by both bodies at the time the constraint is created.</param>
		/// <param name="axisA">The axis fixed to the first body which the second body can rotate around.</param>
		/// <param name="axisB">The axis fixed to the second body which the first body can rotate around.</param>
		/// <param name="aMin">The minimum angle that the second body must maintain relative to the first around the first body's axis.
		/// This number should be zero or negative.</param>
		/// <param name="aMax">The maximum angle that the second body must maintain relative to the first around the first body's axis.
		/// This number should be zero or positive.</param>
		/// <param name="bMin">The minimum angle that the first body must maintain relative to the second around the second body's axis.
		/// This number should be zero or negative.</param>
		/// <param name="bMax">The maximum angle that the first body must maintain relative to the second around the second body's axis.
		/// This number should be zero or positive.</param>
		public UniversalJoint(RigidBody bodyA, RigidBody bodyB, Vector3 worldPoint, Vector3 axisA, Vector3 axisB,
			float aMin, float aMax, float bMin, float bMax)
			: base(bodyA, bodyB)
		{
			aMin = MathHelper.Clamp(aMin, -MathHelper.PiOver2 + Constants.Epsilon, 0f);
			aMax = MathHelper.Clamp(aMax, 0f, MathHelper.PiOver2 - Constants.Epsilon);
			bMin = MathHelper.Clamp(bMin, -MathHelper.PiOver2 + Constants.Epsilon, 0f);
			bMax = MathHelper.Clamp(bMax, 0f, MathHelper.PiOver2 - Constants.Epsilon);

			var frame = new Frame(Vector3.Zero, axisA, axisB, worldPoint);

			this.Constraints.Add(new PointConstraint(bodyA, bodyB, worldPoint));
			this.Constraints.Add(new GenericConstraint(bodyA, bodyB, frame,
				Axes.None, Vector3.Zero, Vector3.Zero,
				Axes.All, new Vector3(0f, aMin, bMin), new Vector3(0f, aMax, bMax)));
		}

		/// <summary>
		/// Construct a new universal joint.
		/// </summary>
		/// <param name="bodyA">The first body to constrain.</param>
		/// <param name="bodyB">The second body to constrain.</param>
		/// <param name="worldPoint">The point in world coordinates shared by both bodies at the time the constraint is created.</param>
		/// <param name="axisA">The axis fixed to the first body which the second body can rotate around.</param>
		/// <param name="axisB">The axis fixed to the second body which the first body can rotate around.</param>
		public UniversalJoint(RigidBody bodyA, RigidBody bodyB, Vector3 worldPoint, Vector3 axisA, Vector3 axisB)
			: this(bodyA, bodyB, worldPoint, axisA, axisB,
			-MathHelper.PiOver2 + 0.01f, MathHelper.PiOver2 - 0.01f,
			-MathHelper.PiOver2 + 0.01f, MathHelper.PiOver2 - 0.01f)
		{
		}
	}
}
