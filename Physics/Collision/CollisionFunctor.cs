using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

using Henge3D.Collision;

namespace Henge3D.Collision
{
	/// <summary>
	/// Provides default implementations and abstract method signatures for a functor that will receive collision information from
	/// the broad phase and narrow phase collision detection implementations. By default, output from the broad phase will be
	/// channelled into the narrow phase routines.
	/// </summary>
	public abstract class CollisionFunctor
	{
		private Dictionary<ulong, NarrowPhase> _tests;

		/// <summary>
		/// Default constructor. Creates the functor and registers narrow phase detection implementations.
		/// </summary>
		public CollisionFunctor()
		{
			_tests = new Dictionary<ulong, NarrowPhase>();

			this.RegisterTest(typeof(SpherePart), typeof(SpherePart), new SphereSphere());

			this.RegisterTest(typeof(SpherePart), typeof(PlanePart), new SpherePlane());
			this.RegisterTest(typeof(CapsulePart), typeof(PlanePart), new CapsulePlane());
			this.RegisterTest(typeof(PolyhedronPart), typeof(PlanePart), new PolyhedronPlane());

			this.RegisterTest(typeof(SpherePart), typeof(PolyhedronPart), new SpherePolyhedron());
			this.RegisterTest(typeof(CapsulePart), typeof(PolyhedronPart), new CapsulePolyhedron());
			this.RegisterTest(typeof(PolyhedronPart), typeof(PolyhedronPart), new PolyhedronPolyhedron());

			this.RegisterTest(typeof(SpherePart), typeof(CapsulePart), new SphereCapsule());
			this.RegisterTest(typeof(CapsulePart), typeof(CapsulePart), new CapsuleCapsule());

			this.RegisterTest(typeof(SpherePart), typeof(MeshPart), new SphereMesh());
			this.RegisterTest(typeof(CapsulePart), typeof(MeshPart), new CapsuleMesh());
			this.RegisterTest(typeof(PolyhedronPart), typeof(MeshPart), new PolyhedronMesh());
		}

		/// <summary>
		/// When called from a broad phase detection implementation, provides a pair of potentially colliding compositions for further
		/// consideration by a narrow phase implementation.
		/// </summary>
		/// <param name="a">The first composition in the potential collision.</param>
		/// <param name="b">The second composition in the potential collision.</param>
		public virtual void WritePair(Composition a, Composition b)
		{
			for (int i = 0; i < a.Count; i++)
			{
				for (int j = 0; j < b.Count; j++)
				{
					ulong key1 = ((ulong)a[i].GetType().GetHashCode() << sizeof(int) * 8) + (ulong)b[j].GetType().GetHashCode();
					ulong key2 = ((ulong)b[j].GetType().GetHashCode() << sizeof(int) * 8) + (ulong)a[i].GetType().GetHashCode();

					if (_tests.ContainsKey(key1))
					{
						this.Test(_tests[key1], a[i], b[j]);
					}
					else if (_tests.ContainsKey(key2))
					{
						this.Test(_tests[key2], b[j], a[i]);
					}
					else
					{
						System.Diagnostics.Debug.WriteLine("No collision test registered for types " + a.GetType().FullName + " and " + b.GetType().FullName);
					}
				}
			}
		}

		internal virtual void WritePair(TaskParams parameters)
		{
			this.WritePair((Composition)parameters.Param1, (Composition)parameters.Param2);
		}

		/// <summary>
		/// When overridden in a derived class and called from a narrow phase detection implementation, registers a single collision
		/// point between two composition parts.
		/// </summary>
		/// <param name="pointA">The collision point on the first part, in world space..</param>
		/// <param name="pointB">The collision point on the second part, in world space.</param>
		/// <param name="normal">The collision normal, in world space.</param>
		public abstract void WritePoint(ref Vector3 pointA, ref Vector3 pointB, ref Vector3 normal);

		protected virtual void Test(NarrowPhase test, Part a, Part b)
		{
			test.OverlapTest(this, a, b);
		}

		protected void RegisterTest(Type a, Type b, NarrowPhase test)
		{
			ulong key = ((ulong)a.GetHashCode() << sizeof(int) * 8) + (ulong)b.GetHashCode();
			if (_tests.ContainsKey(key))
				_tests[key] = test;
			else
				_tests.Add(key, test);
		}
	}
}
