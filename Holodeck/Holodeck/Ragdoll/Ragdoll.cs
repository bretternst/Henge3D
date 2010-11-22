using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Henge3D.Physics;

namespace Henge3D.Holodeck
{
	class Ragdoll : Contraption
	{
		private ScaledSphere _head;
		private ScaledCapsule _torso, _upperLeftArm, _lowerLeftArm, _upperRightArm, _lowerRightArm,
			_upperLeftLeg, _lowerLeftLeg, _upperRightLeg, _lowerRightLeg;
		private Vector3[] _refPositions;
		private Quaternion[] _refOrientations;

		public Ragdoll(Game game)
		{
			_head = new ScaledSphere(game, 0.09f, Color.Yellow.ToVector3());
			_head.SetWorld(new Vector3(0f, 0f, 0.9f));
			this.Bodies.Add(_head);

			_torso = new ScaledCapsule(game, 0.275f, 0.1f, Color.Red.ToVector3());
			_torso.SetWorld(new Vector3(0f, 0f, 0.6f));
			this.Bodies.Add(_torso);

			this.Constraints.Add(new UniversalJoint(_head, _torso, new Vector3(0f, 0f, 0.81f), Vector3.UnitY, Vector3.UnitX,
				-MathHelper.ToRadians(30f), MathHelper.ToRadians(45f),
				-MathHelper.ToRadians(30f), MathHelper.ToRadians(30f)));

			_upperLeftArm = new ScaledCapsule(game, 0.2f, 0.04f, Color.Red.ToVector3());
			_upperLeftArm.SetWorld(new Vector3(0f, 0.175f, 0.75f),
				Quaternion.CreateFromAxisAngle(Vector3.UnitX, MathHelper.PiOver2));
			this.Bodies.Add(_upperLeftArm);

			this.Constraints.Add(new UniversalJoint(_upperLeftArm, _torso, new Vector3(0f, 0.1f, 0.75f), -Vector3.UnitX,  -Vector3.UnitZ,
				-MathHelper.ToRadians(45f), MathHelper.ToRadians(90f),
				-MathHelper.ToRadians(20f), MathHelper.ToRadians(80f)));

			_lowerLeftArm = new ScaledCapsule(game, 0.2f, 0.04f, Color.Red.ToVector3());
			_lowerLeftArm.SetWorld(new Vector3(0f, 0.325f, 0.75f),
				Quaternion.CreateFromAxisAngle(Vector3.UnitX, MathHelper.PiOver2));
			this.Bodies.Add(_lowerLeftArm);

			this.Constraints.Add(new RevoluteJoint(_lowerLeftArm, _upperLeftArm, new Vector3(0f, 0.25f, 0.75f), Vector3.UnitZ,
				-MathHelper.PiOver2, 0f));

			_upperRightArm = new ScaledCapsule(game, 0.2f, 0.04f, Color.Red.ToVector3());
			_upperRightArm.SetWorld(new Vector3(0f, -0.175f, 0.75f),
				Quaternion.CreateFromAxisAngle(Vector3.UnitX, -MathHelper.PiOver2));
			this.Bodies.Add(_upperRightArm);

			this.Constraints.Add(new UniversalJoint(_upperRightArm, _torso, new Vector3(0f, -0.1f, 0.75f), Vector3.UnitX, Vector3.UnitZ,
				-MathHelper.ToRadians(45f), MathHelper.ToRadians(90f),
				-MathHelper.ToRadians(20f), MathHelper.ToRadians(80f)));

			_lowerRightArm = new ScaledCapsule(game, 0.2f, 0.04f, Color.Red.ToVector3());
			_lowerRightArm.SetWorld(new Vector3(0f, -0.325f, 0.75f),
				Quaternion.CreateFromAxisAngle(Vector3.UnitX, -MathHelper.PiOver2));
			this.Bodies.Add(_lowerRightArm);

			this.Constraints.Add(new RevoluteJoint(_lowerRightArm, _upperRightArm, new Vector3(0f, -0.25f, 0.75f), Vector3.UnitZ,
				0f, MathHelper.PiOver2));

			_upperLeftLeg = new ScaledCapsule(game, 0.2f, 0.05f, Color.Blue.ToVector3());
			_upperLeftLeg.SetWorld(new Vector3(0f, 0.075f, 0.35f));
			this.Bodies.Add(_upperLeftLeg);

			this.Constraints.Add(new UniversalJoint(_upperLeftLeg, _torso, new Vector3(0f, 0.075f, 0.45f), Vector3.UnitX, Vector3.UnitY,
				-MathHelper.PiOver4, MathHelper.ToRadians(15f),
				-MathHelper.PiOver2, 0f));
			
			_lowerLeftLeg = new ScaledCapsule(game, 0.2f, 0.05f, Color.Blue.ToVector3());
			_lowerLeftLeg.SetWorld(new Vector3(0f, 0.075f, 0.15f));
			this.Bodies.Add(_lowerLeftLeg);

			this.Constraints.Add(new RevoluteJoint(_lowerLeftLeg, _upperLeftLeg, new Vector3(0f, 0.075f, 0.25f), Vector3.UnitY,
				0f, MathHelper.ToRadians(100f)));

			_upperRightLeg = new ScaledCapsule(game, 0.2f, 0.05f, Color.Blue.ToVector3());
			_upperRightLeg.SetWorld(new Vector3(0f, -0.075f, 0.35f));
			this.Bodies.Add(_upperRightLeg);

			this.Constraints.Add(new UniversalJoint(_torso, _upperRightLeg, new Vector3(0f, -0.075f, 0.45f), Vector3.UnitX, -Vector3.UnitY,
				-MathHelper.PiOver4, MathHelper.ToRadians(15f),
				-MathHelper.PiOver2, 0f));

			_lowerRightLeg = new ScaledCapsule(game, 0.2f, 0.05f, Color.Blue.ToVector3());
			_lowerRightLeg.SetWorld(new Vector3(0f, -0.075f, 0.15f));
			this.Bodies.Add(_lowerRightLeg);

			this.Constraints.Add(new RevoluteJoint(_lowerRightLeg, _upperRightLeg, new Vector3(0f, -0.075f, 0.25f), Vector3.UnitY,
				0f, MathHelper.ToRadians(100f)));

			_refPositions = new Vector3[this.Bodies.Count];
			_refOrientations = new Quaternion[this.Bodies.Count];
			for(int i = 0; i < this.Bodies.Count; i++)
			{
				var body = this.Bodies[i];
				_refPositions[i] = body.Transform.Position;
				_refOrientations[i] = body.Transform.Orientation;
				body.IsActive = false;
			}
			foreach (var constraint in this.Constraints)
			{
				constraint.IsCollisionEnabled = false;
			}
		}

		public void SetWorld(Vector3 position)
		{
			this.SetWorld(position, Quaternion.Identity);
		}

		public void SetWorld(Vector3 position, Quaternion orientation)
		{
			for (int i = 0; i < this.Bodies.Count; i++)
			{
				this.Bodies[i].SetWorld(Vector3.Transform(_refPositions[i], orientation) + position, orientation * _refOrientations[i]);
			}
		}

		public void SetLinearVelocity(Vector3 velocity)
		{
			for (int i = 0; i < this.Bodies.Count; i++)
			{
				this.Bodies[i].LinearVelocity = velocity;
			}
		}

		public void Activate()
		{
			foreach (var rb in this.Bodies)
				rb.IsActive = true;
		}
	}
}
