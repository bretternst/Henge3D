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
				if(_items[i].Count < 1)
				{
					continue;
				}

				int pos = 0;
				for (int j = 0; j < _items[i].Count; j++)
				{
					var c = _items[i][j];
					if (c.BodyA.IsActive || c.BodyB.IsActive ||
						c.BodyA.Manager != _manager || c.BodyB.Manager != _manager)
					{
						_alloc[i].Recycle(c);
					}
					else
					{
						_items[i][pos++] = _items[i][j];
					}
				}
				if(pos < _items[i].Count)
				{
					_items[i].RemoveRange(pos, _items[i].Count - pos);
				}
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

					this.UpdateContactStates(c.BodyA, c.BodyB);
					this.UpdateContactStates(c.BodyB, c.BodyA);

					c.IsCollisionSuppressed = (c.BodyA.ContactStates != null && (c.BodyA.ContactStates[c.BodyB] & ContactStateFlags.IsSuppressed) > 0) ||
						(c.BodyB.ContactStates != null && (c.BodyB.ContactStates[c.BodyA] & ContactStateFlags.IsSuppressed) > 0);
				}
			}
		}

		private bool UpdateContactStates(RigidBody a, RigidBody b)
		{
			if (b.OnCollision != null || b.OnSeparation != null)
			{
				if (b.ContactStates == null)
				{
					b.ContactStates = new Dictionary<RigidBody, ContactStateFlags>();
				}
				if (!b.ContactStates.ContainsKey(a))
				{
					b.ContactStates.Add(a, 0);
				}
				b.ContactStates[a] |= ContactStateFlags.IsInContact;
				if ((b.ContactStates[a] & ContactStateFlags.WasInContact) == 0)
				{
					if (b.OnCollision != null && b.OnCollision(b, a))
					{
						b.ContactStates[a] |= ContactStateFlags.IsSuppressed;
						return true;
					}
				}
			}
			return false;
		}

		// To support modifying the dictionary's values while enumerating the list of bodies.
		private static List<RigidBody> tempBodies = new List<RigidBody>();

		public void ProcessSeparations(IList<RigidBody> bodies)
		{
			for (int i = 0; i < bodies.Count; i++)
			{
				if (bodies[i].ContactStates != null)
				{
					// AddRange would require IEnumerable<T>, creating garbage...
					foreach (var key in bodies[i].ContactStates.Keys)
					{
						tempBodies.Add(key);
					}
					for (int j = 0; j < tempBodies.Count; j++)
					{
						var key = tempBodies[j];
						var val = bodies[i].ContactStates[key];
						if ((val & ContactStateFlags.WasInContact) > 0 &&
							(val & ContactStateFlags.IsInContact) == 0)
						{
							val = 0;
							if (bodies[i].OnSeparation != null)
							{
								bodies[i].OnSeparation(bodies[i], key);
							}
						}

						if (val == 0)
						{
							// I debated whether to actually remove separating items for fear that changing the dictionary
							// too much would result in re-bucketing and garbage for objects that collide with lots of other objects.
							// Fears may be unfounded. We'll try it for now.
							bodies[i].ContactStates.Remove(key);
						}
						else
						{
							// Shift the IsInContact bit field to WasInContact, leaving IsSuppressed intact
							bodies[i].ContactStates[key] =
								((val & ContactStateFlags.IsSuppressed) |
								((val & ContactStateFlags.IsInContact) > 0 ? ContactStateFlags.WasInContact : 0));
						}
					}
					if (bodies[i].OnCollision == null && bodies[i].OnSeparation == null)
					{
						bodies[i].ContactStates = null;
					}
					tempBodies.Clear();
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
