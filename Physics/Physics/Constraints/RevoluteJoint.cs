using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace Henge3D.Physics
{
	/// <summary>
	/// A revolute joint (or hinge joint) constrains the relative orientation of two bodies to change only on a single axis, and
	/// also constrains the position of the two bodies.
	/// </summary>
	public class RevoluteJoint : Joint
	{
		/// <summary>
		/// Construct a new revolute joint.
		/// </summary>
		/// <param name="bodyA">The first constrained body.</param>
		/// <param name="bodyB">The second constrained body.</param>
		/// <param name="worldPoint">The point in world coordinates shared by both bodies at the time the constraint is created.</param>
		/// <param name="axis">The axis around which to allow relative rotation. Rotation around the other axes will be prevented.</param>
		/// <param name="minAngle">The minimum angle specifies (in radians) the most that the first body can rotate clockwise relative
		/// to the second body around the axis. This number should be zero or negative.</param>
		/// <param name="maxAngle">The maximum angle specifies (in radians) the most that the first body can rotate counter-clockwise relative
		/// to the second body around the axis. This number should be zero or positive.</param>
		public RevoluteJoint(RigidBody bodyA, RigidBody bodyB, Vector3 worldPoint, Vector3 axis, float minAngle, float maxAngle)
			: base(bodyA, bodyB)
		{
			var frame = new Frame(worldPoint);

			// create an arbitrary basis with the axis as X
			Vector3 n = Vector3.UnitZ;
			Vector3.Cross(ref n, ref axis, out n);
			frame.X = axis;
			if (n.LengthSquared() < Constants.Epsilon)
			{
				n = Vector3.UnitY;
				Vector3.Cross(ref axis, ref n, out n);
				frame.Z = n;
			}
			else
				frame.Y = n;
			frame.Normalize();

			this.Constraints.Add(new GenericConstraint(bodyA, bodyB, frame,
				Axes.All, Vector3.Zero, Vector3.Zero,
				Axes.All, new Vector3(minAngle, 0f, 0f), new Vector3(maxAngle, 0f, 0f)));
		}

		/// <summary>
		/// Construct a new revolute joint.
		/// </summary>
		/// <param name="bodyA">The first constrained body.</param>
		/// <param name="bodyB">The second constrained body.</param>
		/// <param name="worldPoint">The point in world coordinates shared by both bodies at the time the constraint is created.</param>
		/// <param name="axis">The axis around which to allow relative rotation. Rotation around the other axes will be prevented.</param>
		public RevoluteJoint(RigidBody bodyA, RigidBody bodyB, Vector3 worldPoint, Vector3 axis)
			: this(bodyA, bodyB, worldPoint, axis, -MathHelper.PiOver2, MathHelper.PiOver2)
		{
		}
	}
}
