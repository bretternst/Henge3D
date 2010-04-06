using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace Henge3D
{
	public enum TriangleFeatureType
	{
		None,
		Vertex,
		Edge,
		Face
	}

	public struct TriangleFeature
	{
		public TriangleFeatureType Type;
		public int Index;
		public float X;
	}

	/// <summary>
	/// Represents a single triangle, defined by three vertices ordered in a counter-clockwise fashion and a surface normal.
	/// </summary>
	public struct Triangle
	{
		public Vector3 V1;
		public Vector3 V2;
		public Vector3 V3;
		public Vector3 Normal;

		/// <summary>
		/// Construct a new triangle. The vertices must be ordered counter-clockwise.
		/// </summary>
		/// <param name="v1">The first vertex.</param>
		/// <param name="v2">The second vertex.</param>
		/// <param name="v3">The third vertex.</param>
		public Triangle(Vector3 v1, Vector3 v2, Vector3 v3)
		{
			V1 = v1;
			V2 = v2;
			V3 = v3;
			Vector3 v12, v13;
			Vector3.Subtract(ref v2, ref v1, out v12);
			Vector3.Subtract(ref v3, ref v1, out v13);
			Vector3.Cross(ref v12, ref v13, out Normal);
			Normal.Normalize();
		}

		/// <summary>
		/// Construct a new triangle. The vertices must be ordered counter-clockwise.
		/// </summary>
		/// <param name="v1">The first vertex.</param>
		/// <param name="v2">The second vertex.</param>
		/// <param name="v3">The third vertex.</param>
		/// <param name="normal">The triangle's normal, indicating the winding order of the vertices.</param>
		public Triangle(Vector3 v1, Vector3 v2, Vector3 v3, Vector3 normal)
		{
			V1 = v1;
			V2 = v2;
			V3 = v3;
			Normal = normal;
		}

		/// <summary>
		/// Returns the vertex with the specified index.
		/// </summary>
		/// <param name="index">The index of the vertex: 1, 2, or 3.</param>
		/// <param name="vertex">Returns the requested vertex.</param>
		public void Vertex(int index, out Vector3 vertex)
		{
			switch(index)
			{
				case 1:
					vertex = V1;
					break;
				case 2:
					vertex = V2;
					break;
				case 3:
					vertex = V3;
					break;
				default:
					throw new ArgumentException("Invalid triangle vertex index.", "index");
			}
		}

		/// <summary>
		/// Gets the centroid of the triangle, or the average of all vertices.
		/// </summary>
		/// <param name="p">Returns the centroid.</param>
		public void Center(out Vector3 p)
		{
			p = V1;
			Vector3.Add(ref p, ref V2, out p);
			Vector3.Add(ref p, ref V3, out p);
			Vector3.Multiply(ref p, 1f / 3f, out p);
		}

		/// <summary>
		/// Gets a triangle edge.
		/// </summary>
		/// <param name="index">The index of the triangle edge, or the first vertex that makes up the edge: 1, 2 or 3.</param>
		/// <param name="edge">Returns the edge line segment.</param>
		public void Edge(int index, out Segment edge)
		{
			switch (index)
			{
				case 1:
					edge.P1 = V1;
					edge.P2 = V2;
					break;
				case 2:
					edge.P1 = V2;
					edge.P2 = V3;
					break;
				case 3:
					edge.P1 = V3;
					edge.P2 = V1;
					break;
				default:
					throw new ArgumentException("Invalid triangle edge index.", "index");
			}
		}

		/// <summary>
		/// Gets a triangle edge's direction and magnitude in the form of a vector.
		/// </summary>
		/// <param name="index">The index of the triangle edge, or the first vertex that makes up the edge: 1, 2 or 3.</param>
		/// <param name="edge">Returns the direction and magnitude of the edge.</param>
		public void EdgeVector(int index, out Vector3 edge)
		{
			switch(index)
			{
				case 1:
					Vector3.Subtract(ref V2, ref V1, out edge);
					break;
				case 2:
					Vector3.Subtract(ref V3, ref V2, out edge);
					break;
				case 3:
					Vector3.Subtract(ref V1, ref V3, out edge);
					break;
				default:
					throw new ArgumentException("Invalid triangle edge index.", "index");
			}
		}

		/// <summary>
		/// Determines whether the specified point, when projected onto the triangle's plane, is contained within the triangle.
		/// </summary>
		/// <param name="p">The point to test for containment.</param>
		/// <returns>Returns a value indicating whether the point is contained within the triangle.</returns>
		public bool Contains(ref Vector3 p)
		{
			Vector3 v1, v2, v3;
			Vector3.Subtract(ref V1, ref p, out v1);
			Vector3.Subtract(ref V2, ref p, out v2);
			Vector3.Subtract(ref V3, ref p, out v3);
			float d12, d13, d23, d33, d22;
			Vector3.Dot(ref v1, ref v2, out d12);
			Vector3.Dot(ref v1, ref v3, out d13);
			Vector3.Dot(ref v2, ref v3, out d23);
			Vector3.Dot(ref v3, ref v3, out d33);

			if (d23 * d13 - d33 * d12 < 0f)
				return false;
			Vector3.Dot(ref v2, ref v2, out d22);
			if (d12 * d23 - d13 * d22 < 0f)
				return false;

			return true;
		}

		/// <summary>
		/// Gets the triangle vertex that is at the most extreme along the specified direction.
		/// </summary>
		/// <param name="d">A normalized direction vector.</param>
		/// <returns>Returns the most extreme vertex.</returns>
		public TriangleFeature ExtremeVertex(ref Vector3 d)
		{
			var feature = new TriangleFeature { Type = TriangleFeatureType.Vertex };
			float d1, d2, d3;
			Vector3.Dot(ref d, ref V1, out d1);
			Vector3.Dot(ref d, ref V2, out d2);
			Vector3.Dot(ref d, ref V3, out d3);
			if (d1 >= d2 && d1 >= d3)
			{ feature.Index = 1; feature.X = d1; }
			else if (d2 >= d1 && d2 >= d3)
			{ feature.Index = 2; feature.X = d2; }
			else
			{ feature.Index = 3; feature.X = d3; }
			return feature;
		}

		/// <summary>
		/// Gets the triangle feature that is at the most extreme along the specified direction.
		/// </summary>
		/// <remarks>
		/// If two vertices are equally extreme, then an edge feature is returned.
		/// </remarks>
		/// <param name="d">A normalized direction vector.</param>
		/// <returns>Returns the most extreme feature.</returns>
		public TriangleFeature ExtremeFeature(ref Vector3 d)
		{
			float d1, d2, d3;
			Vector3.Dot(ref d, ref V1, out d1);
			Vector3.Dot(ref d, ref V2, out d2);
			Vector3.Dot(ref d, ref V3, out d3);
			TriangleFeature feature;
			feature.Type = TriangleFeatureType.None;
			feature.Index = -1;
			feature.X = 0f;

			if (Math.Abs(d1 - d2) < Constants.Epsilon && Math.Abs(d2 - d3) < Constants.Epsilon)
			{ feature.Type = TriangleFeatureType.Face; feature.X = d1; }
			else if (Math.Abs(d1 - d2) < Constants.Epsilon && d1 > d3 && d2 > d3)
			{ feature.Type = TriangleFeatureType.Edge; feature.Index = 1; feature.X = d1; }
			else if (Math.Abs(d2 - d3) < Constants.Epsilon && d2 > d1 && d3 > d1)
			{ feature.Type = TriangleFeatureType.Edge; feature.Index = 2; feature.X = d2; }
			else if (Math.Abs(d3 - d1) < Constants.Epsilon && d3 > d2 && d1 > d2)
			{ feature.Type = TriangleFeatureType.Edge; feature.Index = 3; feature.X = d3; }
			else if (d1 >= d2 && d1 >= d3)
			{ feature.Type = TriangleFeatureType.Vertex; feature.Index = 1; feature.X = d1; }
			else if (d2 >= d1 && d2 >= d3)
			{ feature.Type = TriangleFeatureType.Vertex; feature.Index = 2; feature.X = d2; }
			else if (d3 >= d2 && d3 >= d1)
			{ feature.Type = TriangleFeatureType.Vertex; feature.Index = 3; feature.X = d3; }

			return feature;
		}

		/// <summary>
		/// Gets the triangle feature that is at the most extreme along the specified direction and at least as far along
		/// as the specified depth.
		/// </summary>
		/// <remarks>
		/// If two vertices are at least as deep as the specified depth, then an edge feature is returned.
		/// </remarks>
		/// <param name="d">A normalized direction vector.</param>
		/// <param name="depth">The minimum depth for all vertices that make up the extreme feature.</param>
		/// <returns>Returns the most extreme feature.</returns>
		public TriangleFeature ExtremeFeature(ref Vector3 d, float depth)
		{
			float d1, d2, d3;
			Vector3.Dot(ref d, ref V1, out d1);
			Vector3.Dot(ref d, ref V2, out d2);
			Vector3.Dot(ref d, ref V3, out d3);
			TriangleFeature feature;
			feature.Type = TriangleFeatureType.None;
			feature.Index = -1;
			feature.X = depth;

			if (d1 >= depth && d2 >= depth && d3 >= depth)
			{ feature.Type = TriangleFeatureType.Face; }
			else if (d1 >= depth && d2 >= depth)
			{ feature.Type = TriangleFeatureType.Edge; feature.Index = 1; }
			else if (d2 >= depth && d3 >= depth)
			{ feature.Type = TriangleFeatureType.Edge; feature.Index = 2; }
			else if (d3 >= depth && d1 >= depth)
			{ feature.Type = TriangleFeatureType.Edge; feature.Index = 3; }
			else if (d1 >= depth)
			{ feature.Type = TriangleFeatureType.Vertex; feature.Index = 1; }
			else if (d2 >= depth)
			{ feature.Type = TriangleFeatureType.Vertex; feature.Index = 2; }
			else if (d3 >= depth)
			{ feature.Type = TriangleFeatureType.Vertex; feature.Index = 3; }

			return feature;
		}

		/// <summary>
		/// Intersects a line segment with the shape and returns the intersection point closest to the beginning of the segment.
		/// </summary>
		/// <param name="segment">The line segment to intersect.</param>
		/// <param name="scalar">A value between 0 and 1 indicating how far along the segment the intersection occurs.</param>
		/// <param name="p">The point of the intersection closest to the beginning of the line segment.</param>
		/// <returns>Returns a value indicating whether there is an intersection.</returns>
		public bool Intersect(ref Segment seg, out float scalar, out Vector3 p)
		{
			scalar = float.NaN;
			p = Vector3.Zero;
			bool switched = false;
			Segment backup = seg;

			Vector3 v12, v13, p21, v1p1, e, n;
			float d, t, v, w;
			Vector3.Subtract(ref V2, ref V1, out v12);
			Vector3.Subtract(ref V3, ref V1, out v13);
			Vector3.Subtract(ref seg.P1, ref seg.P2, out p21);
			Vector3.Cross(ref v12, ref v13, out n);

			Vector3.Dot(ref n, ref p21, out d);
			if (Math.Abs(d) < Constants.Epsilon)
				return false; // segment is parallel
			else if (d < 0f)
			{
				backup = seg;
				p = seg.P1;
				seg.P1 = seg.P2;
				seg.P2 = p;
				switched = true;
				Vector3.Subtract(ref seg.P1, ref seg.P2, out p21);
				d = -d;
			}

			Vector3.Subtract(ref seg.P1, ref V1, out v1p1);
			Vector3.Dot(ref n, ref v1p1, out t);
			Vector3.Cross(ref p21, ref v1p1, out e);
			Vector3.Dot(ref v13, ref e, out v);
			if (v < 0f || v > d)
				return false; // intersects outside triangle
			Vector3.Dot(ref v12, ref e, out w);
			w = -w;
			if (w < 0f || v + w > d)
				return false; // intersects outside triangle

			d = 1f / d;
			t *= d;
			v *= d;
			w *= d;
			scalar = t;

			// compute intersect point from barycentric coordinates
			Vector3.Multiply(ref v12, v, out v12);
			Vector3.Multiply(ref v13, w, out v13);
			Vector3.Add(ref V1, ref v12, out p);
			Vector3.Add(ref p, ref v13, out p);

			if (switched)
			{
				seg = backup;
				scalar = 1 - scalar;
			}
			return scalar >= 0f && scalar <= 1f;
		}

		/// <summary>
		/// Gets the closest point on the triangle to the specified point.
		/// </summary>
		/// <remarks>
		/// The result can either be any of the vertices, on one of the edges, or another point within the interior of the triangle.
		/// </remarks>
		/// <param name="p">The point against which to find the closest triangle point.</param>
		/// <param name="output">Returns the point on the triangle that is closest.</param>
		/// <returns>Returns a value indicating whether the closest point is in the interior of the triangle (not on any vertex or edge).</returns>
		public bool ClosestPointTo(ref Vector3 p, out Vector3 output)
		{
			float v, w;
			Vector3 v12, v13, v23, v1p, v2p, v3p;
			Vector3.Subtract(ref V2, ref V1, out v12);
			Vector3.Subtract(ref V3, ref V1, out v13);
			Vector3.Subtract(ref p, ref V1, out v1p);

			float d1, d2;
			Vector3.Dot(ref v12, ref v1p, out d1);
			Vector3.Dot(ref v13, ref v1p, out d2);
			if (d1 <= 0f && d2 <= 0f)
			{
				output = V1; // closest point is vertex 1
				return false;
			}

			Vector3.Subtract(ref p, ref V2, out v2p);
			float d3, d4;
			Vector3.Dot(ref v12, ref v2p, out d3);
			Vector3.Dot(ref v13, ref v2p, out d4);
			if (d3 >= 0f && d4 <= d3)
			{
				output = V2; // closest point is vertex 2
				return false;
			}

			float vc = d1 * d4 - d3 * d2;
			if (vc <= 0f && d1 >= 0f && d3 <= 0f)
			{
				// closest point is along edge 1-2
				v = d1 / (d1 - d3);
				Vector3.Multiply(ref v12, v, out output);
				Vector3.Add(ref V1, ref output, out output);
				return false;
			}

			Vector3.Subtract(ref p, ref V3, out v3p);
			float d5, d6;
			Vector3.Dot(ref v12, ref v3p, out d5);
			Vector3.Dot(ref v13, ref v3p, out d6);
			if (d6 >= 0f && d5 <= d6)
			{
				output = V3; // closest point is vertex 3
				return false;
			}

			float vb = d5 * d2 - d1 * d6;
			if (vb <= 0f && d2 >= 0f && d6 <= 0f)
			{
				// closest point is along edge 1-3
				w = d2 / (d2 - d6);
				Vector3.Multiply(ref v13, w, out output);
				Vector3.Add(ref V1, ref output, out output);
				return false;
			}

			float va = d3 * d6 - d5 * d4;
			if (va <= 0f && (d4 - d3) >= 0f && (d5 - d6) >= 0f)
			{
				// closest point is along edge 2-3
				w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
				Vector3.Subtract(ref V3, ref V2, out v23);
				Vector3.Multiply(ref v23, w, out output);
				Vector3.Add(ref V2, ref output, out output);
				return false;
			}

			float denom = 1f / (va + vb + vc);
			v = vb * denom;
			w = vc * denom;
			Vector3.Multiply(ref v12, v, out v12);
			Vector3.Multiply(ref v13, w, out v13);
			Vector3.Add(ref V1, ref v12, out output);
			Vector3.Add(ref output, ref v13, out output);

			return true;
		}

		/// <summary>
		/// Gets the closest point on the triangle to the specified segment.
		/// </summary>
		/// <remarks>
		/// The result can either be any of the vertices, on one of the edges, or another point within the interior of the triangle.
		/// </remarks>
		/// <param name="seg">The segment against which to find the closest triangle point.</param>
		/// <param name="segPoint">Returns the point on the segment that is closest.</param>
		/// <param name="triPoint">Returns the point on the triangle that is closest.</param>
		/// <returns>Returns a value indicating whether the closest point is in the interior of the triangle (not on any vertex or edge).</returns>
		public bool ClosestPointTo(ref Segment seg, out float scalar, out Vector3 segPoint, out Vector3 triPoint)
		{
			// segment intersects triangle
			if (this.Intersect(ref seg, out scalar, out triPoint))
			{
				segPoint = triPoint;
				return true;
			}

			Vector3 v;
			float minDist = float.MaxValue, dtri;
			bool p1inside, p2inside;
			segPoint = Vector3.Zero;
			Vector3.Dot(ref Normal, ref V1, out dtri);

			p1inside = this.Contains(ref seg.P1);
			p2inside = this.Contains(ref seg.P2);

			// both points inside triangle
			if (p1inside && p2inside)
			{
				float d1, d2;
				Vector3.Dot(ref Normal, ref seg.P1, out d1);
				Vector3.Dot(ref Normal, ref seg.P2, out d2);
				d1 -= dtri;
				d2 -= dtri;
				if (Math.Abs(d2 - d1) < Constants.Epsilon)
				{
					// segment is parallel to triangle
					minDist = d1;
					Vector3.Add(ref seg.P1, ref seg.P2, out segPoint);
					Vector3.Multiply(ref segPoint, 0.5f, out segPoint);
				}
				else if (Math.Abs(d1) < Math.Abs(d2))
				{
					segPoint = seg.P1;
					minDist = d1;
					scalar = 0f;
				}
				else
				{
					segPoint = seg.P2;
					minDist = d2;
					scalar = 1f;
				}
				Vector3.Multiply(ref Normal, -minDist, out v);
				Vector3.Add(ref segPoint, ref v, out triPoint);

				return true;
			}
			else if (p1inside)
				segPoint = seg.P1;
			else if (p2inside)
				segPoint = seg.P2;

			// one point is inside triangle
			if (p1inside || p2inside)
			{
				Vector3.Dot(ref Normal, ref segPoint, out minDist);
				minDist -= dtri;
				Vector3.Multiply(ref Normal, -minDist, out v);
				Vector3.Add(ref segPoint, ref v, out triPoint);
				minDist = Math.Abs(minDist);
				minDist *= minDist;
				scalar = p1inside ? 0f : 1f;
			}

			float sa, sb, dist;
			Vector3 pa, pb;

			// test edge 1
			var edge = new Segment(V1, V2);
			Segment.ClosestPoints(ref seg, ref edge, out sa, out pa, out sb, out pb);
			Vector3.DistanceSquared(ref pa, ref pb, out dist);
			if (dist < minDist)
			{
				minDist = dist;
				scalar = sa;
				segPoint = pa;
				triPoint = pb;
			}

			// test edge 2
			edge.P1 = V2;
			edge.P2 = V3;
			Segment.ClosestPoints(ref seg, ref edge, out sa, out pa, out sb, out pb);
			Vector3.DistanceSquared(ref pa, ref pb, out dist);
			if (dist < minDist)
			{
				minDist = dist;
				scalar = sa;
				segPoint = pa;
				triPoint = pb;
			}

			// test edge 3
			edge.P1 = V3;
			edge.P2 = V1;
			Segment.ClosestPoints(ref seg, ref edge, out sa, out pa, out sb, out pb);
			Vector3.DistanceSquared(ref pa, ref pb, out dist);
			if (dist < minDist)
			{
				minDist = dist;
				scalar = sa;
				segPoint = pa;
				triPoint = pb;
			}

			return false;
		}

		public static void Transform(ref Triangle tri, ref Transform transform, out Triangle output)
		{
			Vector3.Transform(ref tri.V1, ref transform.Combined, out output.V1);
			Vector3.Transform(ref tri.V2, ref transform.Combined, out output.V2);
			Vector3.Transform(ref tri.V3, ref transform.Combined, out output.V3);
			Vector3.Transform(ref tri.Normal, ref transform.Orientation, out output.Normal);
		}
	}
}
