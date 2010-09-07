using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace Henge3D.Physics
{
	public struct ContactPoint
	{
		public Vector3 OffsetA;
		public Vector3 OffsetB;
		public float Depth;

		public float NormalMass;
		public float Target;
		public float Impulse;
		public float PositionImpulse;

		public Vector3 Tangent;
		public float TangentMass;
		public float TangentImpulse;
	}

	/// <summary>
	/// A constraint that attempts to prevent inter-penetration between two objects. A contact constraint is generated for each
	/// pair of objects that are colliding.
	/// </summary>
	public sealed class ContactConstraint : Constraint, IRecyclable
	{
		#region Allocator class

		internal class Allocator : Pool<ContactConstraint>
		{
			int maxPoints;

			public Allocator(Game game, int capacity, int growthFactor, int maxPoints)
				: base(capacity, growthFactor)
			{
				this.maxPoints = maxPoints;
			}

			protected override ContactConstraint Create()
			{
				return new ContactConstraint(maxPoints);
			}
		}

		#endregion

		private float _restitution, _friction;
		private int _count;
		private ContactPoint[] _points;
		private Vector3 _normal;

		/// <summary>
		/// Construct a new contact constraint.
		/// </summary>
		public ContactConstraint()
			: base(null, null)
		{
		}

		/// <summary>
		/// Construct a new contact constraint.
		/// </summary>
		/// <param name="maxPoints">The maximum number of supported contact points.</param>
		public ContactConstraint(int maxPoints) : this()
		{
			_points = new ContactPoint[maxPoints];
		}

		/// <summary>
		/// Gets the number of contact points between the two colliding bodies.
		/// </summary>
		public int Count { get { return _count; } set { _count = value; } }

		internal ContactPoint[] Points { get { return _points; } }
		internal Vector3 Normal { get { return _normal; } set { _normal = value; } }
		internal bool IsCollisionSuppressed { get; set; }

		internal void SetParameters(RigidBody a, RigidBody b, 
			float restitution, float friction)
		{
			this.BodyA = a;
			this.BodyB = b;
			_restitution = restitution;
			_friction = friction;
		}

		internal void AddPoint(ref Vector3 offsetA, ref Vector3 offsetB)
		{
			if(BodyA == null || BodyB == null) throw new InvalidOperationException();
			if (_count == _points.Length) return;

			_points[_count++] = new ContactPoint
			{
				OffsetA = offsetA,
				OffsetB = offsetB
			};
		}

		/// <summary>
		/// Prepare the contact constraint for iterative processing within a single frame. Computes the desired target velocities
		/// to attempt to prevent inter-penetration.
		/// </summary>
		public override void PreProcess()
		{
			if (this.IsCollisionSuppressed)
			{
				return;
			}

			RigidBody a = BodyA, b = BodyB;
			var cached = b.Manager.ContactCache.Get(a, b);

			bool isWarmStarted = cached != null && cached.Count == _count;

			// calculate relative force applied during this frame
			Vector3 va = Vector3.Zero, vb = Vector3.Zero;
			float forceMag;
			if (a.IsMovable)
				Vector3.Multiply(ref a.Force, this.Manager.TimeStep * a.Mass.MassInverse, out va);
			if (b.IsMovable)
				Vector3.Multiply(ref b.Force, this.Manager.TimeStep * b.Mass.MassInverse, out vb);
			Vector3.Add(ref va, ref vb, out va);
			Vector3.Dot(ref _normal, ref va, out forceMag);
			forceMag = MathHelper.Min(forceMag, 0f);

			for (int i = 0; i < _count; i++)
			{
				var p = _points[i];

				// calculate movement along normal and tangent
				float normalDelta;
				a.GetVelocityAtPoint(ref p.OffsetA, out va);
				b.GetVelocityAtPoint(ref p.OffsetB, out vb);
				Vector3.Subtract(ref va, ref vb, out va);
				Vector3.Dot(ref _normal, ref va, out normalDelta);

				float tangentDelta;
				Vector3.Multiply(ref _normal, normalDelta, out p.Tangent);
				Vector3.Subtract(ref va, ref p.Tangent, out p.Tangent);
				Vector3.Dot(ref p.Tangent, ref va, out tangentDelta);
				if(p.Tangent.LengthSquared() >= Constants.Epsilon)
				{
					p.Tangent.Normalize();
					Vector3.Negate(ref p.Tangent, out p.Tangent);
				}
				else p.Tangent = Vector3.Zero;

				// calculate effective mass along tangent and normal
				p.NormalMass = MassProperties.EffectiveMass(ref a.MassWorld, ref b.MassWorld, ref p.OffsetA, ref p.OffsetB, ref _normal);
				p.TangentMass = p.Tangent != Vector3.Zero ?
					MassProperties.EffectiveMass(ref a.MassWorld, ref b.MassWorld, ref p.OffsetA, ref p.OffsetB, ref p.Tangent) : 0f;

				// calculate target velocity
				float restitution = Math.Max(_restitution * -(normalDelta - forceMag), 0f);
				float penetration = (p.Depth - this.Manager.LinearErrorTolerance);
				if (restitution < this.Manager.MinRestitution)
				{
					if(penetration > 0f)
						p.Target = penetration * this.Manager.PenetrationBias;
					else
					{
						float scale = MathHelper.Clamp(-0.1f * penetration / this.Manager.LinearErrorTolerance, Constants.Epsilon, 1f);
						p.Target = scale * (p.Depth - this.Manager.LinearErrorTolerance) * this.Manager.TimeStepInverse;
					}
				}
				else
					p.Target = Math.Max(Math.Max(penetration * this.Manager.PenetrationBias, 0f), restitution);

				p.Impulse = 0f;
				p.TangentImpulse = 0f;
				p.PositionImpulse = 0f;

				if (isWarmStarted)
				{
					Vector3 impulse, tangentImpulse;

					// find the best cached point
					var bestPoint = new CachedContactPoint();
					float bestDistance = float.MaxValue;
					for (int j = 0; j < cached.Points.Length; j++)
					{
						float d1, d2;
						Vector3.DistanceSquared(ref cached.Points[j].OffsetA, ref p.OffsetA, out d1);
						Vector3.DistanceSquared(ref cached.Points[j].OffsetB, ref p.OffsetB, out d2);
						if (d1 + d2 < bestDistance)
						{
							bestDistance = d1 + d2;
							bestPoint = cached.Points[j];
						}
					}

					p.Impulse = bestPoint.NormalImpulse;
					float tangentImpulseMag = MathHelper.Clamp(-tangentDelta * p.TangentMass, 0f, _friction * p.Impulse);
					p.TangentImpulse = tangentImpulseMag * p.NormalMass;

					if (Math.Abs(p.Impulse) >= Constants.Epsilon)
					{
						Vector3.Multiply(ref _normal, p.Impulse, out impulse);
						Vector3.Multiply(ref p.Tangent, p.TangentImpulse, out tangentImpulse);
						Vector3.Add(ref impulse, ref tangentImpulse, out impulse);
						a.ApplyImpulse(ref impulse, ref p.OffsetA);
						Vector3.Negate(ref impulse, out impulse);
						b.ApplyImpulse(ref impulse, ref p.OffsetB);
					}
				}
				_points[i] = p;
			}

			// calculate an averaged contact point for help with stabilization during position correction
			if (_count > 2 && _count < _points.Length)
			{
				var ap = _points[_count];
				ap.Depth = float.MaxValue;
				for (int i = 0; i < _count; i++)
				{
					var p = _points[i];

					float depth;
					Vector3 pa, pb;
					Vector3.Add(ref a.World.Position, ref p.OffsetA, out pa);
					Vector3.Add(ref b.World.Position, ref p.OffsetB, out pb);
					Vector3.Add(ref ap.OffsetA, ref pa, out ap.OffsetA);
					Vector3.Add(ref ap.OffsetB, ref pb, out ap.OffsetB);
					Vector3.Subtract(ref pb, ref pa, out pa);
					Vector3.Dot(ref _normal, ref pa, out depth);
					if (depth < ap.Depth)
						ap.Depth = depth;
				}
				Vector3.Divide(ref ap.OffsetA, _count, out ap.OffsetA);
				Vector3.Divide(ref ap.OffsetB, _count, out ap.OffsetB);
				ap.NormalMass = MassProperties.EffectiveMass(ref a.MassWorld, ref b.MassWorld, ref ap.OffsetA, ref ap.OffsetB, ref _normal);
				ap.PositionImpulse = 0f;
				_points[_count] = ap;
			}
		}

		/// <summary>
		/// Solves the constraint for velocity by applying impulses.
		/// </summary>
		public override void ProcessVelocity()
		{
			if (this.IsCollisionSuppressed)
			{
				return;
			}

			RigidBody a = BodyA, b = BodyB;

			for (int i = 0; i < _count; i++)
			{
				var p = _points[i];
				float normalMag;
				Vector3 va, vb;

				float oldImpulseMag, impulseMag;
				Vector3 impulse = Vector3.Zero;

				// step 1: collision impulse
				a.GetVelocityAtPoint(ref p.OffsetA, out va);
				b.GetVelocityAtPoint(ref p.OffsetB, out vb);
				va.X -= vb.X; va.Y -= vb.Y; va.Z -= vb.Z; //Vector3.Subtract(ref va, ref vb, out va);
				normalMag = _normal.X * va.X + _normal.Y * va.Y + _normal.Z * va.Z; //Vector3.Dot(ref normal, ref va, out normalMag);
				impulseMag = (p.Target - normalMag) * p.NormalMass;

				if (Math.Abs(impulseMag) >= Constants.Epsilon)
				{
					oldImpulseMag = p.Impulse;
					p.Impulse = Math.Max(p.Impulse + impulseMag, 0f);
					impulseMag = p.Impulse - oldImpulseMag;
					impulse.X = _normal.X * impulseMag; impulse.Y = _normal.Y * impulseMag; impulse.Z = _normal.Z * impulseMag; // Vector3.Multiply(ref normal, reactImpulse, out impulse);
					a.ApplyImpulse(ref impulse, ref p.OffsetA);
					impulse.X = -impulse.X; impulse.Y = -impulse.Y; impulse.Z = -impulse.Z; // Vector3.Negate(ref impulse, out impulse);
					b.ApplyImpulse(ref impulse, ref p.OffsetB);
				}

				// step 2: friction impulse
				if (p.TangentMass >= Constants.Epsilon)
				{
					float tangentMag;
					a.GetVelocityAtPoint(ref p.OffsetA, out va);
					b.GetVelocityAtPoint(ref p.OffsetB, out vb);
					va.X -= vb.X; va.Y -= vb.Y; va.Z -= vb.Z; // Vector3.Subtract(ref va, ref vb, out va);
					tangentMag = p.Tangent.X * va.X + p.Tangent.Y * va.Y + p.Tangent.Z * va.Z; // Vector3.Dot(ref p.Tangent, ref va, out tangentMag);

					float maxFriction = _friction * p.Impulse;
					float tangentImpulse = -tangentMag * p.TangentMass;
					oldImpulseMag = p.TangentImpulse;
					p.TangentImpulse = MathHelper.Clamp(p.TangentImpulse + tangentImpulse, -maxFriction, maxFriction);
					impulseMag = p.TangentImpulse - oldImpulseMag;

					impulse.X = p.Tangent.X * impulseMag; impulse.Y = p.Tangent.Y * impulseMag; impulse.Z = p.Tangent.Z * impulseMag; // Vector3.Multiply(ref p.Tangent, tangentImpulse, out impulse);
					a.ApplyImpulse(ref impulse, ref p.OffsetA);
					impulse.X = -impulse.X; impulse.Y = -impulse.Y; impulse.Z = -impulse.Z; // Vector3.Negate(ref impulse, out impulse);
					b.ApplyImpulse(ref impulse, ref p.OffsetB);
				}

				_points[i] = p;
			}
		}

		/// <summary>
		/// Solve the constraint for position by applying flash impulses that modify each bodies' position.
		/// </summary>
		/// <returns>Returns a value indicating whether the constraint has been satisfied.</returns>
		public override bool ProcessPosition()
		{
			if (this.IsCollisionSuppressed)
			{
				return true;
			}

			RigidBody a = BodyA, b = BodyB;

			bool satisfied = true;
			for (int i = (_count > 2 && _count < _points.Length ? _count : _count - 1); i >= 0; i--)
			{
				var p = _points[i];

				float depth;
				Vector3 pa, pb;
				Vector3.Add(ref a.World.Position, ref p.OffsetA, out pa);
				Vector3.Add(ref b.World.Position, ref p.OffsetB, out pb);
				Vector3.Subtract(ref pb, ref pa, out pa);
				Vector3.Dot(ref _normal, ref pa, out depth);

				if (Math.Abs(depth) > 2f * this.Manager.LinearErrorTolerance)
					satisfied = false;
				else if (depth <= this.Manager.LinearErrorTolerance)
					continue;

				float impulseMag = Math.Max(depth - this.Manager.LinearErrorTolerance, 0f)
					* this.Manager.PositionCorrectionFactor * p.NormalMass;

				float oldImpulseMag = p.PositionImpulse;
				p.PositionImpulse = MathHelper.Max(oldImpulseMag + impulseMag, 0f);
				impulseMag = p.PositionImpulse - oldImpulseMag;

				Vector3 impulse;
				Vector3.Multiply(ref _normal, impulseMag, out impulse);
				a.ApplyFlashImpulse(ref impulse, ref p.OffsetA);
				Vector3.Negate(ref impulse, out impulse);
				b.ApplyFlashImpulse(ref impulse, ref p.OffsetB);
			}
			return satisfied;
		}

		/// <summary>
		/// Resets the constraint to default values.
		/// </summary>
		public override void Recycle()
		{
			base.Recycle();

			_normal = Vector3.Zero;
			_count = 0;
			_restitution = _friction = 0f;
			this.IsCollisionSuppressed = false;
		}
	}
}
