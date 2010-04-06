using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;
using Henge3D.Collision;

namespace Henge3D.Physics
{
	internal sealed class BodyCollisionFunctor : CollisionFunctor
	{
		private PhysicsManager _manager;
		private TaskManager _taskManager;
		private IAllocator<ContactConstraint>[] _alloc;
		private List<ContactConstraint>[] _items;
		private ContactConstraint[] _contact;
		private Part[] _a;
		private Part[] _b;
		private float _fastSpeedSquared;

		public BodyCollisionFunctor(PhysicsManager manager)
		{
			_manager = manager;
			_taskManager = TaskManager.Current;

			int threads = TaskManager.ThreadCount;

			_alloc = new IAllocator<ContactConstraint>[threads];
			_items = new List<ContactConstraint>[threads];
			_contact = new ContactConstraint[threads];
			_a = new Part[threads];
			_b = new Part[threads];

			for (int i = 0; i < threads; i++)
			{
				_alloc[i] = new ContactConstraint.Allocator(manager.Game, manager.ContactPoolCapacity, 2, manager.MaxPointsPerContact);
				_items[i] = new List<ContactConstraint>();
			}

			_fastSpeedSquared = manager.SweepThresholdSquared;
		}

		public override void WritePair(Composition a, Composition b)
		{
			RigidBody ba = ((BodySkin)a).Owner, bb = ((BodySkin)b).Owner;

			// only process collisions if:
			// - at least one body is active
			// - bodies do not share any constraints on which collisions are disabled
			for (int i = 0; i < bb.Constraints.Count; i++)
			{
				var c = bb.Constraints[i];
				if (!c.IsCollisionEnabled && (c.BodyA == ba || c.BodyB == ba))
					return;
			}

			if (ba.IsActive || bb.IsActive)
			{
				base.WritePair(a, b);
			}
		}

		protected override void Test(NarrowPhase test, Part a, Part b)
		{
			int thread = TaskManager.CurrentThreadIndex;
			_a[thread] = a;
			_b[thread] = b;

			RigidBody ba = ((BodySkin)a.Owner).Owner, bb = ((BodySkin)b.Owner).Owner;
			if (ba.IsFast || bb.IsFast)
			{
				var delta = Vector3.Zero;
				if (ba.IsFast)
				{
					Vector3.Add(ref delta, ref ba.Velocity.Linear, out delta);
				}
				if (bb.IsFast)
				{
					Vector3.Subtract(ref delta, ref bb.Velocity.Linear, out delta);
				}
				Vector3.Multiply(ref delta, bb.Manager.TimeStep, out delta);
				test.SweptTest(this, a, b, delta);
			}
			else
			{
				test.OverlapTest(this, a, b);
			}

			var current = _contact[thread];
			if (current != null)
			{
				var points = current.Points;
				var normal = current.Normal;

				// normalize contact normal if required
				if (Math.Abs(normal.LengthSquared() - 1f) >= Constants.Epsilon)
				{
					normal.Normalize();
					current.Normal = normal;
				}

				for (int i = 0; i < current.Count; i++)
				{
					// transform points to old position
					current.BodyA.Skin.UndoTransform(ref points[i].OffsetA);
					current.BodyB.Skin.UndoTransform(ref points[i].OffsetB);

					// calculate penetration depth of each point
					float ax, bx;
					Vector3.Dot(ref normal, ref points[i].OffsetA, out ax);
					Vector3.Dot(ref normal, ref points[i].OffsetB, out bx);
					points[i].Depth = bx - ax;

					// convert world points to world offsets
					Vector3.Subtract(ref points[i].OffsetA, ref current.BodyA.World.Position, out points[i].OffsetA);
					Vector3.Subtract(ref points[i].OffsetB, ref current.BodyB.World.Position, out points[i].OffsetB);
				}

				_items[thread].Add(current);
				_contact[thread] = null;
			}
		}

		public override void WritePoint(ref Vector3 pointA, ref Vector3 pointB, ref Vector3 normal)
		{
			int thread = TaskManager.CurrentThreadIndex;
			var current = _contact[thread];
			if (current == null)
			{
				current = _contact[thread] = _alloc[thread].Allocate();
				current.Manager = _manager;

				var a = _a[thread];
				var b = _b[thread];
				var bodyA = ((BodySkin)a.Owner).Owner;
				var bodyB = ((BodySkin)b.Owner).Owner;

				// initialize the contact constraint
				var aMat = bodyA.Skin.Material(a);
				var bMat = bodyB.Skin.Material(b);
				current.SetParameters(bodyA, bodyB,
					MathHelper.Clamp(Math.Min(aMat.Elasticity, bMat.Elasticity), 0f, 1f),
					(float)Math.Sqrt(aMat.Roughness * bMat.Roughness));

				// activate any colliding objects
				if (!bodyA.IsActive) bodyA.IsActive = true;
				if (!bodyB.IsActive) bodyB.IsActive = true;
			}

			var points = current.Points;
			int count = current.Count;
			var oldNormal = current.Normal;

			if (count == points.Length)
				return;

			Vector3.Add(ref oldNormal, ref normal, out oldNormal);
			current.Normal = oldNormal;

			for (int i = 0; i < count; i++)
			{
				Vector3 tmp;
				Vector3.Subtract(ref current.Points[i].OffsetA, ref pointA, out tmp);
				if (tmp.LengthSquared() < Constants.Epsilon)
					return;
				Vector3.Subtract(ref current.Points[i].OffsetB, ref pointB, out tmp);
				if (tmp.LengthSquared() < Constants.Epsilon)
					return;
			}

			current.Points[count].OffsetA = pointA;
			current.Points[count].OffsetB = pointB;
			current.Count++;
		}

		public void Clear()
		{
			for (int i = 0; i < _items.Length; i++)
			{
				for (int j = 0; j < _items[i].Count; j++)
				{
					var c = _items[i][j];
					if (c.BodyA.IsActive || c.BodyB.IsActive ||
						c.BodyA.Manager != _manager || c.BodyB.Manager != _manager)
						_alloc[i].Recycle(c);
				}
				_items[i].RemoveAll((c) => c.BodyA == null || c.BodyB == null);
			}
		}

		public void PropagateContacts()
		{
			for (int i = 0; i < _items.Length; i++)
			{
				for (int j = 0; j < _items[i].Count; j++)
				{
					var c = _items[i][j];
					c.BodyA.Contacts.Add(c);
					c.BodyB.Contacts.Add(c);
					if (c.BodyA.IsActive && !c.BodyB.IsActive && c.BodyB.IsMovable) Activate(c.BodyB);
					if (!c.BodyA.IsActive && c.BodyA.IsMovable && c.BodyB.IsActive) Activate(c.BodyA);
				}
			}
		}

		public void PopulateCache(ContactCache cache)
		{
			cache.Clear();
			for (int i = 0; i < _items.Length; i++)
			{
				for (int j = 0; j < _items[i].Count; j++)
				{
					cache.Add(_items[i][j]);
				}
			}
		}

		private void Activate(RigidBody body)
		{
			body.IsActive = true;
			for (int i = 0; i < body.Constraints.Count; i++)
			{
				var c = body.Constraints[i];
				if (!c.BodyA.IsActive && c.BodyA.IsMovable) Activate(body.Constraints[i].BodyA);
				if (!c.BodyB.IsActive && c.BodyB.IsMovable) Activate(body.Constraints[i].BodyB);
			}
			for (int i = 0; i < body.Contacts.Count; i++)
			{
				var c = body.Contacts[i];
				if (!c.BodyA.IsActive && c.BodyA.IsMovable) Activate(c.BodyA);
				if (!c.BodyB.IsActive && c.BodyB.IsMovable) Activate(c.BodyB);
			}
		}
	}
}
