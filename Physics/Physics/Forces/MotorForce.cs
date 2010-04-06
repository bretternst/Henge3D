using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace Henge3D.Physics
{
	/// <summary>
	/// Generates angular force (torque) on a list of bodies.
	/// </summary>
	public class MotorForce : IForceGenerator
	{
		private RigidBody _body;
		private Vector3 _torque;
		private bool _isEnabled = true;

		/// <summary>
		/// Construct a new motor force.
		/// </summary>
		/// <param name="body">The body on which to apply torque.</param>
		/// <param name="worldTorque">The axis and magnitude of the torque in world-space.</param>
		public MotorForce(RigidBody body, Vector3 worldTorque)
		{
			_body = body;
			Vector3.Transform(ref worldTorque, ref _body.WorldInverse.Orientation, out _torque);
		}

		/// <summary>
		/// Gets or sets the body on which torque is applied.
		/// </summary>
		public RigidBody Body { get { return _body; } set { _body = value; } }

		/// <summary>
		/// Gets or sets the axis and magnitude of torque in local-space to be applied.
		/// </summary>
		public Vector3 BodyTorque { get { return _torque; } set { _torque = value; } }

		/// <summary>
		/// Gets or sets a value indicating whether torque should be applied.
		/// </summary>
		public bool IsEnabled { get { return _isEnabled; } set { _isEnabled = value; } }

		/// <summary>
		/// Apply forces to bodies.
		/// </summary>
		/// <param name="bodies">The list of bodies to which forces will be applied.</param>
		public void Generate(IList<RigidBody> bodies)
		{
			if (_isEnabled)
			{
				Vector3 worldTorque;
				Vector3.Transform(ref _torque, ref _body.World.Orientation, out worldTorque);
				_body.ApplyTorque(ref worldTorque);
			}
		}
	}
}
