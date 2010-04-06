using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace Henge3D.Physics
{
	/// <summary>
	/// Generates a force that acts like a spring, varying the force by the distance between two bodies.
	/// </summary>
	public class SpringForce : IForceGenerator
	{
		private RigidBody _bodyA, _bodyB;
		private Vector3 _bodyPointA, _bodyPointB;
		private float _length, _k, _c;

		/// <summary>
		/// Construct a new spring force.
		/// </summary>
		/// <param name="bodyA">The first body on which force is applied.</param>
		/// <param name="bodyB">The second body on which force is applied.</param>
		/// <param name="bodyPointA">The point in local-space on the first body on which to apply force.</param>
		/// <param name="bodyPointB">The point in local-space on the second body on which to apply force.</param>
		/// <param name="length">The resting length of the spring.</param>
		/// <param name="stiffness">The stiffness factor of the spring.</param>
		/// <param name="damping">The damping coefficient.</param>
		public SpringForce(RigidBody bodyA, RigidBody bodyB, Vector3 bodyPointA, Vector3 bodyPointB, float length, float stiffness, float damping)
		{
			_bodyA = bodyA;
			_bodyB = bodyB;
			_bodyPointA = bodyPointA;
			_bodyPointB = bodyPointB;
			_length = length;
			_k = stiffness;
			_c = damping;
		}

		/// <summary>
		/// Gets or sets the first body on which the force is applied.
		/// </summary>
		public RigidBody BodyA { get { return _bodyA; } set { _bodyA = value; } }

		/// <summary>
		/// Gets or sets the second body on which the force is applied.
		/// </summary>
		public RigidBody BodyB { get { return _bodyB; } set { _bodyB = value; } }

		/// <summary>
		/// Gets the point in local-space on the first body to which force is applied.
		/// </summary>
		public Vector3 BodyPointA { get { return _bodyPointA; } set { _bodyPointA = value; } }

		/// <summary>
		/// Gets the point in local-space on the second body to which force is applied.
		/// </summary>
		public Vector3 BodyPointB { get { return _bodyPointB; } set { _bodyPointB = value; } }

		/// <summary>
		/// Gets or sets the length of the spring.
		/// </summary>
		public float Length { get { return _length; } set { _length = value; } }

		/// <summary>
		/// Gets or sets the stiffness factor of the spring.
		/// </summary>
		public float Stiffness { get { return _k; } set { _k = value; } }

		/// <summary>
		/// Gets or sets the damping coefficient of the spring.
		/// </summary>
		public float Damping { get { return _c; } set { _c = value; } }

		/// <summary>
		/// Apply forces to bodies.
		/// </summary>
		/// <param name="bodies">The list of bodies to which forces will be applied.</param>
		public void Generate(IList<RigidBody> bodies)
		{
			Vector3 pa, pb, n;

			Vector3.Transform(ref _bodyPointA, ref _bodyA.World.Combined, out pa);
			Vector3.Transform(ref _bodyPointB, ref _bodyB.World.Combined, out pb);
			Vector3.Subtract(ref pa, ref pb, out n);
			Vector3.Subtract(ref pa, ref _bodyA.World.Position, out pa);
			Vector3.Subtract(ref pb, ref _bodyB.World.Position, out pb);

			float dist = n.Length();
			Vector3.Divide(ref n, dist, out n);

			float speed;
			Vector3 va, vb;
			_bodyA.GetVelocityAtPoint(ref pa, out va);
			_bodyB.GetVelocityAtPoint(ref pb, out vb);
			Vector3.Subtract(ref va, ref vb, out va);
			Vector3.Dot(ref n, ref va, out speed);

			dist -= _length;

			Vector3.Multiply(ref n, dist * _k + speed * _c, out n);
			_bodyB.ApplyForce(ref n, ref pb);
			Vector3.Negate(ref n, out n);
			_bodyA.ApplyForce(ref n, ref pa);

			if (n.LengthSquared() >= Constants.Epsilon)
			{
				_bodyA.IsActive = true;
				_bodyB.IsActive = true;
			}
		}
	}
}
