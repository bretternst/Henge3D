using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace Henge3D.Physics
{
	internal struct CachedContactPoint
	{
		public Vector3 OffsetA;
		public Vector3 OffsetB;
		public float NormalImpulse;

		public CachedContactPoint(ContactPoint p)
		{
			OffsetA = p.OffsetA;
			OffsetB = p.OffsetB;
			NormalImpulse = p.Impulse;
		}
	}

	internal class CachedContact
	{
		private int _count;
		private CachedContactPoint[] _points;

		public int Count { get { return _count; } set { _count = value; } }
		public CachedContactPoint[] Points { get { return _points; } set { _points = value; } }

		public CachedContact()
		{
		}

		public CachedContact(int maxPoints)
		{
			_points = new CachedContactPoint[maxPoints];
		}
	}

	internal sealed class ContactCache
	{
		#region Allocator class

		public class Allocator : Pool<CachedContact>
		{
			private int _maxPoints;

			public Allocator(int capacity, int growthFactor, int maxPoints)
				: base(capacity, growthFactor)
			{
				this._maxPoints = maxPoints;
			}

			protected override CachedContact Create()
			{
				return new CachedContact(_maxPoints);
			}
		}

		#endregion

		#region BodyPair struct

		struct BodyPair
		{
			public RigidBody BodyA;
			public RigidBody BodyB;

			public BodyPair(RigidBody a, RigidBody b)
			{
				BodyA = a;
				BodyB = b;
			}

			public override int GetHashCode()
			{
				return ((object)BodyA).GetHashCode() ^ ((object)BodyB).GetHashCode();
			}
		}

		#endregion

		#region BodyPair Comparer

		class BodyPairComparer : IEqualityComparer<BodyPair>
		{
			public bool Equals(BodyPair x, BodyPair y)
			{
				return x.BodyA == y.BodyA && x.BodyB == y.BodyB;
			}

			public int GetHashCode(BodyPair obj)
			{
				return obj.GetHashCode();
			}
		}

		#endregion

		private IAllocator<CachedContact> _alloc;
		private Dictionary<BodyPair, CachedContact> _pairs;
		private List<CachedContact> _allocatedContacts;

		public ContactCache(IAllocator<CachedContact> alloc)
		{
			_alloc = alloc;
			_pairs = new Dictionary<BodyPair,CachedContact>(new BodyPairComparer());
			_allocatedContacts = new List<CachedContact>();
		}

		public void Add(ContactConstraint contact)
		{
			var cachedContact = _alloc.Allocate();
			cachedContact.Count = contact.Count;
			for (int j = 0; j < contact.Count; j++)
			{
				cachedContact.Points[j] = new CachedContactPoint(contact.Points[j]);
			}
			var pair = new BodyPair(contact.BodyA, contact.BodyB);
			if (!_pairs.ContainsKey(pair)) _pairs.Add(pair, cachedContact);
			_allocatedContacts.Add(cachedContact);
		}

		public void Clear()
		{
			_pairs.Clear();
			_alloc.Recycle(_allocatedContacts);
			_allocatedContacts.Clear();
		}

		public CachedContact Get(RigidBody a, RigidBody b)
		{
			var pair = new BodyPair(a, b);
			return _pairs.ContainsKey(pair) ? _pairs[pair] : null;
		}
	}
}
