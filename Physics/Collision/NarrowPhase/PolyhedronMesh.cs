using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

using Henge3D.Pipeline;

namespace Henge3D.Collision
{
	public class PolyhedronMesh : NarrowPhase
	{
		public override void OverlapTest(CollisionFunctor cf, Part partA, Part partB)
		{
			var a = (PolyhedronPart)partA;
			var b = (MeshPart)partB;

			var tf = Functor;
			tf.Initialize(cf, a, b, Vector3.Zero);
			b.ProcessTriangles(tf);

			// if the shape is inside the mesh, push it out via the normal of least depth
			if (tf.Depth > 0f && tf.Depth < float.MaxValue)
			{
				Triangle tri;
				Vector3 pa, pb;
				Triangle.Transform(ref tf.NearestTriangle, ref b.Transform, out tri);
				a.Center(out pa);
				tri.Center(out pb);
				cf.WritePoint(ref pa, ref pb, ref tri.Normal);
			}
		}

		public override void SweptTest(CollisionFunctor cf, Part partA, Part partB, Vector3 delta)
		{
			var a = (PolyhedronPart)partA;
			var b = (MeshPart)partB;

			var tf = Functor;
			tf.Initialize(cf, a, b, delta);
			b.ProcessTriangles(tf);
		}

		private struct CollisionInfo
		{
			public PolyhedronFeature FeatureA;
			public TriangleFeature FeatureB;
			public Vector3 Normal, NormalNeg;
			public float Depth;
		}

		private static void OverlapTest(CollisionFunctor cf, PolyhedronPart a, MeshPart b, ref Triangle tri)
		{
			CollisionInfo cur = new CollisionInfo(), final;
			Vector3 v;
			float d;

			// axis: face normal of B
			cur.Normal = tri.Normal;
			Vector3.Negate(ref cur.Normal, out cur.NormalNeg);
			cur.FeatureA = a.ExtremeVertex(ref cur.NormalNeg);
			cur.FeatureA.X = -cur.FeatureA.X;
			cur.FeatureB.Type = TriangleFeatureType.Face;
			Vector3.Dot(ref cur.Normal, ref tri.V1, out cur.FeatureB.X);
			cur.Depth = cur.FeatureB.X - cur.FeatureA.X;
			if (cur.Depth <= -Constants.Epsilon)
				return;
			final = cur;

			// axes: face normals of A
			for (int i = 0; i < a.FaceCount; i++)
			{
				a.FaceNormal(i, out cur.NormalNeg);
				Vector3.Negate(ref cur.NormalNeg, out cur.Normal);
				a.World(a.Face(i)[0], out v);
				cur.FeatureA = new PolyhedronFeature(PolyhedronFeatureType.Face, i, 0f);
				Vector3.Dot(ref cur.Normal, ref v, out cur.FeatureA.X);
				cur.FeatureB = tri.ExtremeVertex(ref cur.Normal);
				cur.Depth = cur.FeatureB.X - cur.FeatureA.X;

				if (cur.Depth <= -Constants.Epsilon)
					return;
				else if (cur.Depth < final.Depth)
					final = cur;
			}

			// crossed edges from A and B
			Vector3 centerA, centerB;
			a.Center(out centerA);
			tri.Center(out centerB);
			for (int i = 0; i < a.EdgeVectorCount; i++)
			{
				for (int j = 1; j <= 3; j++)
				{
					// calculate normal from the two edge vectors
					Vector3 eva, evb;
					a.EdgeVector(i, out eva);
					tri.EdgeVector(j, out evb);
					Vector3.Cross(ref eva, ref evb, out cur.Normal);
					if (cur.Normal.LengthSquared() < Constants.Epsilon)
						continue;
					cur.Normal.Normalize();

					float ca, cb;
					Vector3.Dot(ref cur.Normal, ref centerA, out ca);
					Vector3.Dot(ref cur.Normal, ref centerB, out cb);
					if (ca < cb)
					{
						cur.NormalNeg = cur.Normal;
						Vector3.Negate(ref cur.NormalNeg, out cur.Normal);
					}
					else
						Vector3.Negate(ref cur.Normal, out cur.NormalNeg);

					// skip this normal if it's close to one we already have
					Vector3.Dot(ref cur.Normal, ref final.Normal, out d);
					if (Math.Abs(1f - d) < Constants.Epsilon)
						continue;

					cur.FeatureA = a.ExtremeVertex(ref cur.NormalNeg);
					cur.FeatureA.X = -cur.FeatureA.X;
					cur.FeatureB = tri.ExtremeVertex(ref cur.Normal);
					cur.Depth = cur.FeatureB.X - cur.FeatureA.X;

					if (cur.Depth <= -Constants.Epsilon)
						return;
					else if (final.Depth - cur.Depth >= Constants.Epsilon)
						final = cur;
				}
			}

			if (final.FeatureA.Type == PolyhedronFeatureType.Vertex)
			{
				final.FeatureA = a.ExtremeFeature(ref final.NormalNeg, -final.FeatureB.X);
				final.FeatureA.X = -final.FeatureA.X;
			}
			if (final.FeatureB.Type == TriangleFeatureType.Vertex)
			{
				final.FeatureB = tri.ExtremeFeature(ref final.Normal, final.FeatureA.X);
			}

			// make sure the normal points outward
			Vector3.Dot(ref tri.Normal, ref final.Normal, out d);
			if (d < 0f)
			{
				Vector3.Multiply(ref tri.Normal, -2f * d, out v);
				Vector3.Add(ref final.Normal, ref v, out final.Normal);
			}

			CalculateContactPoints(cf, a, b, ref tri, ref final);
		}

		private static bool SweptTest(CollisionFunctor cf, PolyhedronPart a, MeshPart b, ref Triangle tri, ref Vector3 delta)
		{
			CollisionInfo cur = new CollisionInfo(), final = new CollisionInfo() { Depth = float.MaxValue };
			Vector3 v;
			float d, dx, curTime, finalTime = 0f, tlast = float.MaxValue;

			// axis: face normal of B
			cur.Normal = tri.Normal;
			Vector3.Negate(ref cur.Normal, out cur.NormalNeg);
			cur.FeatureA = a.ExtremeVertex(ref cur.NormalNeg);
			cur.FeatureA.X = -cur.FeatureA.X;
			cur.FeatureB.Type = TriangleFeatureType.Face;
			Vector3.Dot(ref cur.Normal, ref tri.V1, out cur.FeatureB.X);
			cur.Depth = cur.FeatureB.X - cur.FeatureA.X;
			Vector3.Dot(ref cur.Normal, ref delta, out dx);
			curTime = cur.Depth / dx;

			final = cur;
			if (cur.Depth >= 0f)
			{
				if (dx > 0f)
					tlast = curTime;
			}
			else
			{
				if (dx >= 0f || curTime > 1f)
					return false;
				finalTime = curTime;
			}

			// axes: face normals of A
			for (int i = 0; i < a.FaceCount; i++)
			{
				a.FaceNormal(i, out cur.NormalNeg);
				Vector3.Negate(ref cur.NormalNeg, out cur.Normal);
				a.World(a.Face(i)[0], out v);
				cur.FeatureA = new PolyhedronFeature(PolyhedronFeatureType.Face, i, 0f);
				Vector3.Dot(ref cur.Normal, ref v, out cur.FeatureA.X);
				cur.FeatureB = tri.ExtremeVertex(ref cur.Normal);
				cur.Depth = cur.FeatureB.X - cur.FeatureA.X;
				Vector3.Dot(ref cur.Normal, ref delta, out dx);
				curTime = cur.Depth / dx;

				if (cur.Depth >= 0f)
				{
					if (finalTime <= 0f && cur.Depth < final.Depth)
					{
						final = cur;
						finalTime = 0f;
					}
					if (dx > 0f && curTime < tlast)
						tlast = curTime;
				}
				else
				{
					if (dx >= 0f || curTime > 1f)
						return false;
					if (curTime > finalTime)
					{
						final = cur;
						finalTime = curTime;
					}
				} 
			}

			// crossed edges from A and B
			Vector3 centerA, centerB;
			a.Center(out centerA);
			tri.Center(out centerB);
			for (int i = 0; i < a.EdgeVectorCount; i++)
			{
				for (int j = 1; j <= 3; j++)
				{
					// calculate normal from the two edge vectors
					Vector3 eva, evb;
					a.EdgeVector(i, out eva);
					tri.EdgeVector(j, out evb);
					Vector3.Cross(ref eva, ref evb, out cur.Normal);
					if (cur.Normal.LengthSquared() < Constants.Epsilon)
						continue;
					cur.Normal.Normalize();

					float ca, cb;
					Vector3.Dot(ref cur.Normal, ref centerA, out ca);
					Vector3.Dot(ref cur.Normal, ref centerB, out cb);
					if (ca < cb)
					{
						cur.NormalNeg = cur.Normal;
						Vector3.Negate(ref cur.NormalNeg, out cur.Normal);
					}
					else
						Vector3.Negate(ref cur.Normal, out cur.NormalNeg);

					// skip this normal if it's close to one we already have
					Vector3.Dot(ref cur.Normal, ref final.Normal, out d);
					if (Math.Abs(1f - d) < Constants.Epsilon)
						continue;

					cur.FeatureA = a.ExtremeVertex(ref cur.NormalNeg);
					cur.FeatureA.X = -cur.FeatureA.X;
					cur.FeatureB = tri.ExtremeVertex(ref cur.Normal);
					cur.Depth = cur.FeatureB.X - cur.FeatureA.X;
					Vector3.Dot(ref cur.Normal, ref delta, out dx);
					curTime = cur.Depth / dx;

					if (cur.Depth >= 0f)
					{
						if (finalTime <= 0f && cur.Depth < final.Depth)
						{
							final = cur;
							finalTime = 0f;
						}
						if (dx > 0f && curTime < tlast)
							tlast = curTime;
					}
					else
					{
						if (dx >= 0f || curTime > 1f)
							return false;
						if (curTime > finalTime)
						{
							final = cur;
							finalTime = curTime;
						}
					}
				}
			}

			if (finalTime >= tlast)
				return false;

			Vector3.Dot(ref final.Normal, ref delta, out dx);
			if (finalTime <= 0f)
				dx = 0f;

			if (final.FeatureA.Type == PolyhedronFeatureType.Vertex)
			{
				final.FeatureA = a.ExtremeFeature(ref final.NormalNeg, dx - final.FeatureB.X);
				final.FeatureA.X = -final.FeatureA.X;
			}
			if (final.FeatureB.Type == TriangleFeatureType.Vertex)
			{
				final.FeatureB = tri.ExtremeFeature(ref final.Normal, dx + final.FeatureA.X);
			}

			// make sure the normal points outward
			Vector3.Dot(ref tri.Normal, ref final.Normal, out d);
			if (d < 0f)
			{
				Vector3.Multiply(ref tri.Normal, -2f * d, out v);
				Vector3.Add(ref final.Normal, ref v, out final.Normal);
			}

			CalculateContactPoints(cf, a, b, ref tri, ref final);
			return true;
		}

		private static void CalculateContactPoints(CollisionFunctor cf, PolyhedronPart a, MeshPart b, ref Triangle tri, ref CollisionInfo ci)
		{
			if (ci.FeatureB.Type == TriangleFeatureType.Vertex)
			{
				if (ci.FeatureA.Type == PolyhedronFeatureType.Vertex)
					CalculateVertexVertexPoint(cf, a, b, ref tri, ref ci);
				else if (ci.FeatureA.Type == PolyhedronFeatureType.Edge)
					CalculateEdgeVertexPoint(cf, a, b, ref tri, ref ci);
				else if (ci.FeatureA.Type == PolyhedronFeatureType.Face)
					CalculateFaceVertexPoint(cf, a, b, ref tri, ref ci);
			}
			else if (ci.FeatureB.Type == TriangleFeatureType.Edge)
			{
				if (ci.FeatureA.Type == PolyhedronFeatureType.Vertex)
					CalculateVertexEdgePoint(cf, a, b, ref tri, ref ci);
				else if (ci.FeatureA.Type == PolyhedronFeatureType.Edge)
					CalculateEdgeEdgePoints(cf, a, b, ref tri, ref ci);
				else if (ci.FeatureA.Type == PolyhedronFeatureType.Face)
					CalculateFaceEdgePoints(cf, a, b, ref tri, ref ci);
			}
			else if (ci.FeatureB.Type == TriangleFeatureType.Face)
			{
				if (ci.FeatureA.Type == PolyhedronFeatureType.Vertex)
					CalculateVertexFacePoint(cf, a, b, ref tri, ref ci);
				else if (ci.FeatureA.Type == PolyhedronFeatureType.Edge)
					CalculateEdgeFacePoints(cf, a, b, ref tri, ref ci);
				else if (ci.FeatureA.Type == PolyhedronFeatureType.Face)
					CalculateFaceFacePoints(cf, a, b, ref tri, ref ci);
			}
		}

		private static void CalculateVertexVertexPoint(CollisionFunctor cf, PolyhedronPart a, MeshPart b, ref Triangle tri, ref CollisionInfo ci)
		{
			Vector3 pa, pb;
			a.World(ci.FeatureA.Index, out pa);
			tri.Vertex(ci.FeatureB.Index, out pb);
			cf.WritePoint(ref pa, ref pb, ref ci.Normal);
		}

		private static void CalculateEdgeVertexPoint(CollisionFunctor cf, PolyhedronPart a, MeshPart b, ref Triangle tri, ref CollisionInfo ci)
		{
			Vector3 pa, pb;
			Segment ea;
			float sa;

			int[] edgeA = a.Edge(ci.FeatureA.Index);
			a.World(edgeA[0], out ea.P1);
			a.World(edgeA[1], out ea.P2);
			tri.Vertex(ci.FeatureB.Index, out pb);
			ea.ClosestPointTo(ref pb, out sa, out pa);
			cf.WritePoint(ref pa, ref pb, ref ci.Normal);
		}

		private static void CalculateVertexEdgePoint(CollisionFunctor cf, PolyhedronPart a, MeshPart b, ref Triangle tri, ref CollisionInfo ci)
		{
			Vector3 pa, pb;
			Segment eb;
			float sb;

			tri.Edge(ci.FeatureB.Index, out eb);
			a.World(ci.FeatureA.Index, out pa);
			eb.ClosestPointTo(ref pa, out sb, out pb);
			cf.WritePoint(ref pa, ref pb, ref ci.Normal);
		}

		private static void CalculateFaceVertexPoint(CollisionFunctor cf, PolyhedronPart a, MeshPart b, ref Triangle tri, ref CollisionInfo ci)
		{
			Vector3 pa, pb;

			tri.Vertex(ci.FeatureB.Index, out pb);
			a.World(a.Face(ci.FeatureA.Index)[0], out pa);
			var plane = new Plane(pa, ci.Normal);
			plane.ClosestPointTo(ref pb, out pa);
			cf.WritePoint(ref pa, ref pb, ref ci.Normal);
		}

		private static void CalculateVertexFacePoint(CollisionFunctor cf, PolyhedronPart a, MeshPart b, ref Triangle tri, ref CollisionInfo ci)
		{
			Vector3 pa, pb;

			a.World(ci.FeatureA.Index, out pa);
			var plane = new Plane(tri.V1, ci.Normal);
			plane.ClosestPointTo(ref pa, out pb);
			cf.WritePoint(ref pa, ref pb, ref ci.Normal);
		}

		private static void CalculateEdgeEdgePoints(CollisionFunctor cf, PolyhedronPart a, MeshPart b, ref Triangle tri, ref CollisionInfo ci)
		{
			Vector3 pa, pb;
			Segment ea, eb;

			int[] edgeA = a.Edge(ci.FeatureA.Index);
			a.World(edgeA[0], out ea.P1);
			a.World(edgeA[1], out ea.P2);
			tri.Edge(ci.FeatureB.Index, out eb);

			float sa, sb;
			Segment.ClosestPoints(ref ea, ref eb, out sa, out pa, out sb, out pb);
			cf.WritePoint(ref pa, ref pb, ref ci.Normal);
		}

		private static void CalculateFaceEdgePoints(CollisionFunctor cf, PolyhedronPart a, MeshPart b, ref Triangle tri, ref CollisionInfo ci)
		{
			Vector3 pa, pb;
			Segment ea, eba, eb;
			int[] faceA = a.Face(ci.FeatureA.Index);
			a.World(faceA[0], out pa);
			Plane planeA = new Plane(pa, ci.Normal);
			tri.Edge(ci.FeatureB.Index, out eb);
			planeA.ClosestPointTo(ref eb.P1, out eba.P1);
			planeA.ClosestPointTo(ref eb.P2, out eba.P2);

			int count = 0;
			if (a.IsPointOnFace(ci.FeatureA.Index, ref eba.P1, true))
			{
				count++;
				cf.WritePoint(ref eba.P1, ref eb.P1, ref ci.Normal);
			}
			if (a.IsPointOnFace(ci.FeatureA.Index, ref eba.P2, true))
			{
				count++;
				cf.WritePoint(ref eba.P2, ref eb.P2, ref ci.Normal);
			}

			for(int i = 0; i < faceA.Length && count < 2; i++)
			{
				a.World(faceA[i == 0 ? faceA.Length - 1 : i - 1], out ea.P1);
				a.World(faceA[i], out ea.P2);

				float sa, sb;
				Segment.ClosestPoints(ref ea, ref eb, out sa, out pa, out sb, out pb);
				if (sa > 0f && sa < 1f && sb > 0f && sb < 1f)
				{
					count++;
					cf.WritePoint(ref pa, ref pb, ref ci.Normal);
				}
			}
		}

		private static void CalculateEdgeFacePoints(CollisionFunctor cf, PolyhedronPart a, MeshPart b, ref Triangle tri, ref CollisionInfo ci)
		{
			Vector3 pa, pb;
			Segment ea, eb, eab;

			int[] edge = a.Edge(ci.FeatureA.Index);
			a.World(edge[0], out ea.P1);
			a.World(edge[1], out ea.P2);
			Plane planeB = new Plane(tri.V1, tri.Normal);
			planeB.ClosestPointTo(ref ea.P1, out eab.P1);
			planeB.ClosestPointTo(ref ea.P2, out eab.P2);

			int count = 0;
			if (tri.Contains(ref eab.P1))
			{
				count++;
				cf.WritePoint(ref ea.P1, ref eab.P1, ref ci.Normal);
			}
			if (tri.Contains(ref eab.P2))
			{
				count++;
				cf.WritePoint(ref ea.P2, ref eab.P2, ref ci.Normal);
			}

			for (int i = 1; i <= 3 && count < 2; i++)
			{
				tri.Edge(i, out eb);
				float sa, sb;
				Segment.ClosestPoints(ref ea, ref eb, out sa, out pa, out sb, out pb);
				if (sa > 0f && sa < 1f && sb > 0f && sb < 1f)
				{
					count++;
					cf.WritePoint(ref pa, ref pb, ref ci.Normal);
				}
			}
		}

		private static void CalculateFaceFacePoints(CollisionFunctor cf, PolyhedronPart a, MeshPart b, ref Triangle tri, ref CollisionInfo ci)
		{
			int[] faceA = a.Face(ci.FeatureA.Index);
			Vector3 pa, pb, n;

			a.World(faceA[0], out pa);
			a.FaceNormal(ci.FeatureA.Index, out n);
			Plane planeA = new Plane(pa, n);
			Plane planeB = new Plane(tri.V1, tri.Normal);

			// vertices of A contained in face of B
			for (int i = 0; i < faceA.Length; i++)
			{
				a.World(faceA[i], out pa);
				planeB.ClosestPointTo(ref pa, out pb);
				if (tri.Contains(ref pb))
				{
					cf.WritePoint(ref pa, ref pb, ref ci.Normal);
				}
			}

			for (int i = 1; i <= 3; i++)
			{
				tri.Vertex(i, out pb);
				planeA.ClosestPointTo(ref pb, out pa);
				if (a.IsPointOnFace(ci.FeatureA.Index, ref pa, true))
				{
					cf.WritePoint(ref pa, ref pb, ref ci.Normal);
				}
			}

			// intersection of edges from both faces
			Segment ea, eb;
			for (int i = 0; i < faceA.Length; i++)
			{
				a.World(faceA[i == 0 ? faceA.Length - 1 : i - 1], out ea.P1);
				a.World(faceA[i], out ea.P2);

				for (int j = 1; j <= 3; j++)
				{
					tri.Edge(j, out eb);

					float sa, sb;
					Segment.ClosestPoints(ref ea, ref eb, out sa, out pa, out sb, out pb);
					if (sa > 0f && sa < 1f && sb > 0f && sb < 1f)
					{
						cf.WritePoint(ref pa, ref pb, ref ci.Normal);
					}
				}
			}
		}

		private class PolyhedronMeshFunctor : TriangleFunctor
		{
			private CollisionFunctor _cf;
			private PolyhedronPart _a;
			private MeshPart _b;
			private Vector3 _center, _delta;
			private bool _useSweptTest;

			public float Depth;
			public Triangle NearestTriangle;

			public void Initialize(CollisionFunctor cf, PolyhedronPart a, MeshPart b, Vector3 delta)
			{
				_cf = cf;
				_a = a;
				_b = b;
				_delta = delta;
				_useSweptTest = _delta != Vector3.Zero;
				Depth = float.MaxValue;
				a.Center(out _center);

				// transform bounding box to body space
				b.BoundingBox(out BoundingBox);
				AlignedBox.Transform(ref BoundingBox, ref b.TransformInverse, out BoundingBox);
			}

			public override void Process(int count)
			{
				float ax, bx, d;

				for (int i = 0; i < count; i++)
				{
					Triangle.Transform(ref Buffer[i], ref _b.Transform, out Buffer[i]);

					// ignore triangles that we're behind
					Vector3.Dot(ref Buffer[i].Normal, ref _center, out ax);
					Vector3.Dot(ref Buffer[i].Normal, ref Buffer[i].V1, out bx);
					d = bx - ax;
					if (d > 0f && d < Depth)
						this.NearestTriangle = Buffer[i];
					Depth = Math.Min(d, Depth);
					if (d > 0f)
						continue;

					if (_useSweptTest)
					{
						PolyhedronMesh.SweptTest(_cf, _a, _b, ref Buffer[i], ref _delta);
					}
					else
					{
						PolyhedronMesh.OverlapTest(_cf, _a, _b, ref Buffer[i]);
					}
				}
			}
		}

		private static PolyhedronMeshFunctor Functor
		{
			get
			{
				var tf = _functors[TaskManager.CurrentThreadIndex];
				if (tf == null)
					tf = _functors[TaskManager.CurrentThreadIndex] = new PolyhedronMeshFunctor();
				return tf;
			}
		}

		private static PolyhedronMeshFunctor[] _functors = new PolyhedronMeshFunctor[TaskManager.ThreadCount];
	}
}
