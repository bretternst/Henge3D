using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace Henge3D.Physics
{
	/// <summary>
	/// Generates a force in a specific direction and with a specific magnitude, used primarily for gravity.
	/// </summary>
	public class GravityForce : IForceGenerator
	{
		private Vector3 _gravity;
		private int _gravityAxis;

		/// <summary>
		/// Construct a new gravity force.
		/// </summary>
		/// <param name="gravity">The direction and magnitude of the force to apply.</param>
		public GravityForce(Vector3 gravity)
		{
			_gravity = gravity;
			_gravityAxis = ComputeGravityAxis();
		}

		/// <summary>
		/// Gets or sets the direction and magnitude of the force to apply.
		/// </summary>
		public Vector3 Gravity { get { return _gravity; } set { _gravity = value; _gravityAxis = ComputeGravityAxis(); } }

		/// <summary>
		/// Gets the world axis along which the gravity is most aligned.
		/// </summary>
		public int GravityAxis { get { return _gravityAxis; } }

		/// <summary>
		/// Apply forces to bodies.
		/// </summary>
		/// <param name="bodies">The list of bodies to which forces will be applied.</param>
		public void Generate(IList<RigidBody> bodies)
		{
			Vector3 f;

			for (int i = 0; i < bodies.Count; i++)
			{
				if (!bodies[i].IsWeightless)
				{
					Vector3.Multiply(ref _gravity, bodies[i].Mass.Mass, out f);
					bodies[i].ApplyForce(ref f);
				}
			}
		}

		private int ComputeGravityAxis()
		{
			if (_gravity.Z > _gravity.X && _gravity.Z > _gravity.Y)
				return 3;
			else if (_gravity.Y > _gravity.X)
				return 2;
			else
				return 1;
		}
	}
}
