using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace Henge3D.Physics
{
	/// <summary>
	/// Generates a force pulling objects toward a specific point.
	/// </summary>
	public class SingularityForce : IForceGenerator
	{
		public const float GravitationalConstant = 6.67428E-11f;

		private Vector3 _position;

		/// <summary>
		/// Construct a new singularity force.
		/// </summary>
		/// <param name="gravity">The position of the singularity.</param>
		public SingularityForce(Vector3 position, float mass)
		{
			this.Position = position;
			this.Mass = mass;

			System.Diagnostics.Debug.WriteLine(GravitationalConstant * this.Mass);
		}

		/// <summary>
		/// Gets or sets the position of the singularity.
		/// </summary>
		public Vector3 Position { get { return _position; } set { _position = value; } }

		/// <summary>
		/// Gets or sets the mass of the singularity.
		/// </summary>
		public float Mass { get; set; }

		/// <summary>
		/// Apply forces to bodies.
		/// </summary>
		/// <param name="bodies">The list of bodies to which forces will be applied.</param>
		public void Generate(IList<RigidBody> bodies)
		{
			for (int i = 0; i < bodies.Count; i++)
			{
				if (!bodies[i].IsWeightless)
				{
					Vector3 f;

					Vector3.Subtract(ref _position, ref bodies[i].World.Position, out f);
					float r2 = f.LengthSquared();
					f.Normalize();

					Vector3.Multiply(ref f, (GravitationalConstant * this.Mass * bodies[i].Mass.Mass) / r2, out f);

					bodies[i].ApplyForce(ref f);
				}
			}
		}
	}
}
