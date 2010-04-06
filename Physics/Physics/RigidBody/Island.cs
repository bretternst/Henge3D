using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace Henge3D.Physics
{
	internal sealed class Island : IRecyclable
	{
		#region Constraint Comparator

		private class ConstraintComparator : Comparer<Constraint>
		{
			public override int Compare(Constraint x, Constraint y)
			{
				if (x is ContactConstraint ^ y is ContactConstraint)
				{
					return x is ContactConstraint ? 1 : -1;
				}
				float d = MathHelper.Min(x.BodyA.World.Position.Z, x.BodyB != null ? x.BodyB.World.Position.Z : 0f) -
					MathHelper.Min(y.BodyA.World.Position.Z, y.BodyB != null ? y.BodyB.World.Position.Z : 0f);
				return d == 0f ? 0 : (d > 0f ? 1 : -1);
			}
		}

		private static ConstraintComparator _constraintComparator = new ConstraintComparator();

		#endregion

		private List<RigidBody> _bodies;
		private List<Constraint> _constraints;
		private PhysicsManager _manager;

		public Island()
		{
			_bodies = new List<RigidBody>();
			_constraints = new List<Constraint>();
		}

		public PhysicsManager Manager { get { return _manager; } internal set { _manager = value; } }

		public void Process()
		{
			float dt = Manager.TimeStep;

			// do island deactivation
			if (CanDeactivate())
			{
				Deactivate();
				return;
			}

			if(_manager.IsContactListSorted)
				_constraints.Sort(_constraintComparator);

			// process constraints for velocity
			ProcessVelocityConstraints();

			// integrate position
			for (int i = 0; i < _bodies.Count; i++)
				_bodies[i].IntegrateVelocity(dt);

			// process constraints for position
			ProcessPositionConstraints();
		}

		public void Process(TaskParams parameters)
		{
			this.Process();
		}

		public void Recycle()
		{
			for (int i = 0; i < _bodies.Count; i++) _bodies[i].Island = null;
			for (int i = 0; i < _constraints.Count; i++) _constraints[i].Island = null;
			_bodies.Clear();
			_constraints.Clear();
			Manager = null;
		}

		public void Add(RigidBody a)
		{
			if (a.Island != null || !a.IsMovable) return;

			_bodies.Add(a);
			a.Island = this;

			for (int i = 0; i < a.Constraints.Count; i++)
			{
				var constraint = a.Constraints[i];
				if (constraint.Island == null)
				{
					_constraints.Add(constraint);
					constraint.Island = this;
					var other = constraint.BodyA == a ? constraint.BodyB : constraint.BodyA;
					if (other != null && other.Island == null && other.IsMovable)
						this.Add(other);
				}
			}
			for (int i = 0; i < a.Contacts.Count; i++)
			{
				var constraint = a.Contacts[i];
				if (constraint.Island == null)
				{
					_constraints.Add(constraint);
					constraint.Island = this;
					var other = constraint.BodyA == a ? constraint.BodyB : constraint.BodyA;
					if (other != null && other.Island == null && other.IsMovable)
						this.Add(other);
				}
			}
		}

		public bool CanDeactivate()
		{
			for (int i = 0; i < _bodies.Count; i++)
			{
				if (!_bodies[i].CanDeactivate)
				{
					return false;
				}
			}
			return true;
		}

		public void Deactivate()
		{
			for (int i = 0; i < _bodies.Count; i++)
			{
				_bodies[i].IsActive = false;
			}
		}

		private void ProcessVelocityConstraints()
		{
			bool reverse = false;
			for (int i = 0; i < _constraints.Count; i++)
			{
				var c = _constraints[i];
				c.BodyA.IsActive = true;
				c.BodyB.IsActive = true;
				c.PreProcess();
			}
			for (int i = 0; i < _manager.VelocityIterations; i++)
			{
				for (int j = reverse ? _constraints.Count-1 : 0; reverse && j >= 0 || !reverse && j < _constraints.Count; j += reverse ? -1 : 1)
				{
					_constraints[j].ProcessVelocity();
				}
				reverse = !reverse;
			}
		}

		private void ProcessPositionConstraints()
		{
			bool reverse = false;
			for (int i = 0; i < _manager.PositionIterations; i++)
			{
				bool satisfied = true;
				for (int j = reverse ? _constraints.Count - 1 : 0; reverse && j >= 0 || !reverse && j < _constraints.Count; j += reverse ? -1 : 1)
				{
					satisfied &= _constraints[j].ProcessPosition();
				}
				if (satisfied)
					break;
				reverse = !reverse;
			}
		}
	}
}
