using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace Henge3D.Collision
{
	public class PolyhedronPolyhedron : NarrowPhase
	{
		#region CollisionInfo struct

		private struct CollisionInfo
		{
			public PolyhedronFeature FeatureA, FeatureB;
			public Vector3 Normal, NormalNeg;
			public float Depth;
		}

		#endregion

		public override void OverlapTest(CollisionFunctor cf, Part partA, Part partB)
		{
			var a = (PolyhedronPart)partA;
			var b = (PolyhedronPart)partB;

			CollisionInfo cur = new CollisionInfo(),
				final = new CollisionInfo { Depth = float.PositiveInfinity };

			Vector3 v;

			// axes: face normals of A
			for (int i = 0; i < a.FaceCount; i++)
			{
				a.FaceNormal(i, out cur.NormalNeg);
				Vector3.Negate(ref cur.NormalNeg, out cur.Normal);
				a.World(a.Face(i)[0], out v);
				cur.FeatureA = new PolyhedronFeature(PolyhedronFeatureType.Face, i, 0f);
				Vector3.Dot(ref cur.Normal, ref v, out cur.FeatureA.X);
				cur.FeatureB = b.ExtremeVertex(ref cur.Normal);
				cur.Depth = cur.FeatureB.X - cur.FeatureA.X;

				if (cur.Depth <= -Constants.Epsilon)
					return;
				else if (cur.Depth < final.Depth)
					final = cur;
			}

			// axes: face normals of B
			for (int i = 0; i < b.FaceCount; i++)
			{
				b.FaceNormal(i, out cur.Normal);
				Vector3.Negate(ref cur.Normal, out cur.NormalNeg);
				b.World(b.Face(i)[0], out v);
				cur.FeatureB = new PolyhedronFeature(PolyhedronFeatureType.Face, i, 0f);
				Vector3.Dot(ref cur.Normal, ref v, out cur.FeatureB.X);
				cur.FeatureA = a.ExtremeVertex(ref cur.NormalNeg);
				cur.FeatureA.X = -cur.FeatureA.X;
				cur.Depth = cur.FeatureB.X - cur.FeatureA.X;

				if (cur.Depth <= -Constants.Epsilon)
					return;
				else if (cur.Depth < final.Depth)
					final = cur;
			}

			// axes: crossed edges from A and B
			Vector3 centerA, centerB;
			a.Center(out centerA);
			b.Center(out centerB);
			for (int i = 0; i < a.EdgeVectorCount; i++)
			{
				for (int j = 0; j < b.EdgeVectorCount; j++)
				{
					Vector3 eva, evb;
					a.EdgeVector(i, out eva);
					b.EdgeVector(j, out evb);
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

					// ignore this axis if it's close to the one we already have
					float d;
					Vector3.Dot(ref cur.Normal, ref final.Normal, out d);
					if (Math.Abs(1f - d) < Constants.Epsilon)
						continue;

					cur.FeatureA = a.ExtremeVertex(ref cur.NormalNeg);
					cur.FeatureA.X = -cur.FeatureA.X;
					cur.FeatureB = b.ExtremeVertex(ref cur.Normal);

					cur.Depth = cur.FeatureB.X - cur.FeatureA.X;
					if (cur.Depth <= -Constants.Epsilon)
						return;
					else if (final.Depth - cur.Depth >= Constants.Epsilon)
					{
						final = cur;
					}
				}
			}

			if (final.FeatureA.Type == PolyhedronFeatureType.Vertex)
			{
				final.FeatureA = a.ExtremeFeature(ref final.NormalNeg, -final.FeatureB.X);
				final.FeatureA.X = -final.FeatureA.X;
			}
			if (final.FeatureB.Type == PolyhedronFeatureType.Vertex)
			{
				final.FeatureB = b.ExtremeFeature(ref final.Normal, final.FeatureA.X);
			}

			CalculateContactPoints(cf, a, b, ref final);
		}

		public override void SweptTest(CollisionFunctor cf, Part partA, Part partB, Vector3 delta)
		{
			var a = (PolyhedronPart)partA;
			var b = (PolyhedronPart)partB;

			CollisionInfo cur = new CollisionInfo(),
				final = new CollisionInfo { Depth = float.PositiveInfinity };
			float dx, curTime, finalTime = 0f, lastTime = float.PositiveInfinity;

			Vector3 v;

			// axes: face normals of A
			for (int i = 0; i < a.FaceCount; i++)
			{
				a.FaceNormal(i, out cur.NormalNeg);
				Vector3.Negate(ref cur.NormalNeg, out cur.Normal);
				a.World(a.Face(i)[0], out v);
				cur.FeatureA = new PolyhedronFeature(PolyhedronFeatureType.Face, i, 0f);
				Vector3.Dot(ref cur.Normal, ref v, out cur.FeatureA.X);
				cur.FeatureB = b.ExtremeVertex(ref cur.Normal);
				cur.Depth = cur.FeatureB.X - cur.FeatureA.X;
				Vector3.Dot(ref delta, ref cur.Normal, out dx);
				curTime = cur.Depth / dx;

				if (cur.Depth >= -Constants.Epsilon)
				{
					if(finalTime <= 0f && cur.Depth < final.Depth)
					{
						final = cur;
						finalTime = 0f;
					}
					if(dx > 0f && curTime < lastTime)
						lastTime = curTime;
				}
				else
				{
					if(dx >= 0 || curTime > 1f)
						return;
					if (curTime > finalTime)
					{
						final = cur;
						finalTime = curTime;
					}
				}
			}

			// axes: face normals of B
			for (int i = 0; i < b.FaceCount; i++)
			{
				b.FaceNormal(i, out cur.Normal);
				Vector3.Negate(ref cur.Normal, out cur.NormalNeg);
				b.World(b.Face(i)[0], out v);
				cur.FeatureB = new PolyhedronFeature(PolyhedronFeatureType.Face, i, 0f);
				Vector3.Dot(ref cur.Normal, ref v, out cur.FeatureB.X);
				cur.FeatureA = a.ExtremeVertex(ref cur.NormalNeg);
				cur.FeatureA.X = -cur.FeatureA.X;
				cur.Depth = cur.FeatureB.X - cur.FeatureA.X;
				Vector3.Dot(ref delta, ref cur.Normal, out dx);
				curTime = cur.Depth / dx;

				if (cur.Depth >= -Constants.Epsilon)
				{
					if (finalTime <= 0f && cur.Depth < final.Depth)
					{
						final = cur;
						finalTime = 0f;
					}
					if (dx > 0f && curTime < lastTime)
						lastTime = curTime;
				}
				else
				{
					if (dx >= 0f || curTime > 1f)
						return;
					if (curTime > finalTime)
					{
						final = cur;
						finalTime = curTime;
					}
				}
			}

			// axes: crossed edges from A and B
			Vector3 centerA, centerB;
			a.Center(out centerA);
			b.Center(out centerB);
			for (int i = 0; i < a.EdgeVectorCount; i++)
			{
				for (int j = 0; j < b.EdgeVectorCount; j++)
				{
					Vector3 eva, evb;
					a.EdgeVector(i, out eva);
					b.EdgeVector(j, out evb);
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

					// ignore this axis if it's close to the one we already have
					float d;
					Vector3.Dot(ref cur.Normal, ref final.Normal, out d);
					if (Math.Abs(1f - d) < Constants.Epsilon)
						continue;

					cur.FeatureA = a.ExtremeVertex(ref cur.NormalNeg);
					cur.FeatureA.X = -cur.FeatureA.X;
					cur.FeatureB = b.ExtremeVertex(ref cur.Normal);

					cur.Depth = cur.FeatureB.X - cur.FeatureA.X;
					Vector3.Dot(ref delta, ref cur.Normal, out dx);
					curTime = cur.Depth / dx;

					if (cur.Depth >= -Constants.Epsilon)
					{
						if (finalTime <= 0f && cur.Depth < final.Depth)
						{
							final = cur;
							finalTime = 0f;
						}
						if (dx > 0f && curTime < lastTime)
							lastTime = curTime;
					}
					else
					{
						if (dx >= 0 || curTime > 1f)
							return;
						if (curTime > finalTime)
						{
							final = cur;
							finalTime = curTime;
						}
					}
				}
			}

			if (finalTime >= lastTime)
				return;

			Vector3.Dot(ref final.Normal, ref delta, out dx);
			if (finalTime <= 0f)
			{
				dx = 0f;
				finalTime = 0f;
			}

			if (final.FeatureA.Type == PolyhedronFeatureType.Vertex)
			{
				final.FeatureA = a.ExtremeFeature(ref final.NormalNeg, dx - final.FeatureB.X);
				final.FeatureA.X = -final.FeatureA.X;
			}
			if (final.FeatureB.Type == PolyhedronFeatureType.Vertex)
			{
				final.FeatureB = b.ExtremeFeature(ref final.Normal, dx + final.FeatureA.X);
			}

			CalculateContactPoints(cf, a, b, ref final);
		}

		private static void CalculateContactPoints(CollisionFunctor cf, PolyhedronPart a, PolyhedronPart b, ref CollisionInfo ci)
		{
			if (ci.FeatureA.Type == PolyhedronFeatureType.None || ci.FeatureB.Type == PolyhedronFeatureType.None)
				System.Diagnostics.Debug.WriteLine("Unhandled collision case!");

			// calculate contact manifold
			if (ci.FeatureB.Type == PolyhedronFeatureType.Vertex)
			{
				if (ci.FeatureA.Type == PolyhedronFeatureType.Vertex)
					CalculateVertexVertexPoint(cf, a, b, ref ci);
				else if (ci.FeatureA.Type == PolyhedronFeatureType.Edge)
					CalculateEdgeVertexPoint(cf, a, b, ref ci);
				else if (ci.FeatureA.Type == PolyhedronFeatureType.Face)
					CalculateFaceVertexPoint(cf, a, b, ref ci);
			}
			else if (ci.FeatureB.Type == PolyhedronFeatureType.Edge)
			{
				if (ci.FeatureA.Type == PolyhedronFeatureType.Vertex)
					CalculateVertexEdgePoint(cf, a, b, ref ci);
				else if (ci.FeatureA.Type == PolyhedronFeatureType.Edge)
					CalculateEdgeEdgePoints(cf, a, b, ref ci);
				else if (ci.FeatureA.Type == PolyhedronFeatureType.Face)
					CalculateFaceEdgePoints(cf, a, b, ref ci);
			}
			else if (ci.FeatureB.Type == PolyhedronFeatureType.Face)
			{
				if (ci.FeatureA.Type == PolyhedronFeatureType.Vertex)
					CalculateVertexFacePoint(cf, a, b, ref ci);
				else if (ci.FeatureA.Type == PolyhedronFeatureType.Edge)
					CalculateEdgeFacePoints(cf, a, b, ref ci);
				else if (ci.FeatureA.Type == PolyhedronFeatureType.Face)
					CalculateFaceFacePoints(cf, a, b, ref ci);
			}
		}

		private static void CalculateVertexVertexPoint(CollisionFunctor cf, PolyhedronPart a, PolyhedronPart b, ref CollisionInfo ci)
		{
			Vector3 pa, pb;
			a.World(ci.FeatureA.Index, out pa);
			b.World(ci.FeatureB.Index, out pb);
			cf.WritePoint(ref pa, ref pb, ref ci.Normal);
		}

		private static void CalculateEdgeVertexPoint(CollisionFunctor cf, PolyhedronPart a, PolyhedronPart b, ref CollisionInfo ci)
		{
			Vector3 pa, pb;
			Segment ea;
			float sa;

			int[] edgeA = a.Edge(ci.FeatureA.Index);
			a.World(edgeA[0], out ea.P1);
			a.World(edgeA[1], out ea.P2);
			b.World(ci.FeatureB.Index, out pb);
			ea.ClosestPointTo(ref pb, out sa, out pa);
			cf.WritePoint(ref pa, ref pb, ref ci.Normal);
		}

		private static void CalculateVertexEdgePoint(CollisionFunctor cf, PolyhedronPart a, PolyhedronPart b, ref CollisionInfo ci)
		{
			Vector3 pa, pb;
			Segment eb;
			float sb;

			int[] edgeB = b.Edge(ci.FeatureB.Index);
			b.World(edgeB[0], out eb.P1);
			b.World(edgeB[1], out eb.P2);
			a.World(ci.FeatureA.Index, out pa);
			eb.ClosestPointTo(ref pa, out sb, out pb);
			cf.WritePoint(ref pa, ref pb, ref ci.Normal);
		}

		private static void CalculateFaceVertexPoint(CollisionFunctor cf, PolyhedronPart a, PolyhedronPart b, ref CollisionInfo ci)
		{
			Vector3 pa, pb;

			b.World(ci.FeatureB.Index, out pb);
			a.World(a.Face(ci.FeatureA.Index)[0], out pa);
			var plane = new Plane(pa, ci.Normal);
			plane.ClosestPointTo(ref pb, out pa);
			cf.WritePoint(ref pa, ref pb, ref ci.Normal);
		}

		private static void CalculateVertexFacePoint(CollisionFunctor cf, PolyhedronPart a, PolyhedronPart b, ref CollisionInfo ci)
		{
			Vector3 pa, pb;

			a.World(ci.FeatureA.Index, out pa);
			b.World(b.Face(ci.FeatureB.Index)[0], out pb);
			var plane = new Plane(pb, ci.Normal);
			plane.ClosestPointTo(ref pa, out pb);
			cf.WritePoint(ref pa, ref pb, ref ci.Normal);
		}

		private static void CalculateEdgeEdgePoints(CollisionFunctor cf, PolyhedronPart a, PolyhedronPart b, ref CollisionInfo ci)
		{
			Vector3 pa, pb;
			Segment ea, eb;

			int[] edgeA = a.Edge(ci.FeatureA.Index);
			int[] edgeB = b.Edge(ci.FeatureB.Index);

			a.World(edgeA[0], out ea.P1);
			a.World(edgeA[1], out ea.P2);
			b.World(edgeB[0], out eb.P1);
			b.World(edgeB[1], out eb.P2);

			float sa, sb;
			Segment.ClosestPoints(ref ea, ref eb, out sa, out pa, out sb, out pb);
			cf.WritePoint(ref pa, ref pb, ref ci.Normal);
		}

		private static void CalculateFaceEdgePoints(CollisionFunctor cf, PolyhedronPart a, PolyhedronPart b, ref CollisionInfo ci)
		{
			Vector3 pa, pb;
			Segment ea, eb, eba;
			int[] edgeB = b.Edge(ci.FeatureB.Index);
			b.World(edgeB[0], out eb.P1);
			b.World(edgeB[1], out eb.P2);
			int[] faceA = a.Face(ci.FeatureA.Index);
			a.World(faceA[0], out pa);
			var planeA = new Plane(pa, ci.Normal);
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

		private static void CalculateEdgeFacePoints(CollisionFunctor cf, PolyhedronPart a, PolyhedronPart b, ref CollisionInfo ci)
		{
			Vector3 pa, pb;
			Segment ea, eb, eab;
			int[] edgeA = a.Edge(ci.FeatureA.Index);
			a.World(edgeA[0], out ea.P1);
			a.World(edgeA[1], out ea.P2);
			int[] faceB = b.Face(ci.FeatureB.Index);
			b.World(faceB[0], out pb);
			var planeB = new Plane(pb, ci.Normal);
			planeB.ClosestPointTo(ref ea.P1, out eab.P1);
			planeB.ClosestPointTo(ref ea.P2, out eab.P2);

			int count = 0;
			if (b.IsPointOnFace(ci.FeatureB.Index, ref eab.P1, true))
			{
				count++;
				cf.WritePoint(ref ea.P1, ref eab.P1, ref ci.Normal);
			}
			if (b.IsPointOnFace(ci.FeatureB.Index, ref eab.P2, true))
			{
				count++;
				cf.WritePoint(ref ea.P2, ref eab.P2, ref ci.Normal);
			}

			for (int i = 0; i < faceB.Length && count < 2; i++)
			{
				b.World(faceB[i == 0 ? faceB.Length - 1 : i - 1], out eb.P1);
				b.World(faceB[i], out eb.P2);

				float sa, sb;
				Segment.ClosestPoints(ref ea, ref eb, out sa, out pa, out sb, out pb);
				if (sa > 0f && sa < 1f && sb > 0f && sb < 1f)
				{
					count++;
					cf.WritePoint(ref pa, ref pb, ref ci.Normal);
				}
			}
		}

		private static void CalculateFaceFacePoints(CollisionFunctor cf, PolyhedronPart a, PolyhedronPart b, ref CollisionInfo ci)
		{
			int[] faceA = a.Face(ci.FeatureA.Index);
			int[] faceB = b.Face(ci.FeatureB.Index);

			Vector3 pa, pb, n;

			a.World(faceA[0], out pa);
			a.FaceNormal(ci.FeatureA.Index, out n);
			Plane planeA = new Plane(pa, n);
			b.World(faceB[0], out pb);
			Plane planeB = new Plane(pb, ci.Normal);

			// vertices of A contained in face of B
			for (int i = 0; i < faceA.Length; i++)
			{
				a.World(faceA[i], out pa);
				planeB.ClosestPointTo(ref pa, out pb);
				if(b.IsPointOnFace(ci.FeatureB.Index, ref pb, true))
				{
					cf.WritePoint(ref pa, ref pb, ref ci.Normal);
				}
			}

			// vertices of B contained in face of A
			for (int i = 0; i < faceB.Length; i++)
			{
				b.World(faceB[i], out pb);
				planeA.ClosestPointTo(ref pb, out pa);
				if (a.IsPointOnFace(ci.FeatureA.Index, ref pa, true))
				{
					cf.WritePoint(ref pa, ref pb, ref ci.Normal);
				}
			}

			// intersections of edges from both faces
			Segment ea, eb;
			for (int i = 0; i < faceA.Length; i++)
			{
				a.World(faceA[i == 0 ? faceA.Length - 1 : i - 1], out ea.P1);
				a.World(faceA[i], out ea.P2);

				for (int j = 0; j < faceB.Length; j++)
				{
					b.World(faceB[j == 0 ? faceB.Length - 1 : j - 1], out eb.P1);
					b.World(faceB[j], out eb.P2);

					float sa, sb;
					Segment.ClosestPoints(ref ea, ref eb, out sa, out pa, out sb, out pb);
					if (sa > 0f && sa < 1f && sb > 0f && sb < 1f)
					{
						cf.WritePoint(ref pa, ref pb, ref ci.Normal);
					}
				}
			}
		}
	}
}
