using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Content;

namespace Henge3D
{
	public enum PolyhedronFeatureType
	{
		None,
		Vertex,
		Edge,
		Face
	}

	public struct PolyhedronFeature
	{
		public PolyhedronFeatureType Type;
		public int Index;
		public float X;

		public PolyhedronFeature(PolyhedronFeatureType type, int index, float x)
		{
			Type = type;
			Index = index;
			X = x;
		}
	}

	/// <summary>
	/// A polyhedron collision skin part defined by a convex polyhedron composed of an arbitrary number of vertices and faces.
	/// </summary>
	public sealed class PolyhedronPart : Part
	{
		private CompiledPolyhedron _compiled;
		private Vector3[] _world;
		private Vector3[] _edgeVectors;
		private Vector3[] _faceNormals;
		private Vector3 _center;

		/// <summary>
		/// Construct a polyhedron collision part from the pre-compiled polyhedron.
		/// </summary>
		/// <param name="compiled"></param>
		public PolyhedronPart(CompiledPolyhedron compiled)
		{
			this._compiled = compiled;
			Initialize();
		}

		/// <summary>
		/// Construct a polyhedron collisin part from the specified vertices and faces.
		/// </summary>
		/// <param name="vertices">A list of all vertices in the polyhedron.</param>
		/// <param name="faces">A collection of faces, each defined by an array of indices specifying the vertices that belong to the face.
		/// The vertices are expected to be provided in counter-clockwise order for each face.</param>
		public PolyhedronPart(Vector3[] vertices, int[][] faces)
		{
			_compiled = new CompiledPolyhedron(vertices, faces);
			Initialize();
		}

		/// <summary>
		/// Gets the number of unique edge vectors defining the "direction" of an edge. Note that more than one edge may share a common
		/// edge vector if they are parallel, so this count can be less than the total number of edges.
		/// </summary>
		public int EdgeVectorCount { get { return _edgeVectors.Length; } }

		/// <summary>
		/// Gets the total number of faces that make up the polyhedron.
		/// </summary>
		public int FaceCount { get { return _faceNormals.Length; } }

		/// <summary>
		/// Gets the total number of edges that make up the polyhedron.
		/// </summary>
		public int EdgeCount { get { return _compiled.Edges.Length; } }

		private void Initialize()
		{
			_world = new Vector3[_compiled.Body.Length];
			this._edgeVectors = new Vector3[_compiled.EdgeVectors.Length];
			this._faceNormals = new Vector3[_compiled.FaceNormals.Length];
		}

		/// <summary>
		/// Gets the center of the polyhedron in world-space as defined by the average of all vertices.
		/// </summary>
		/// <param name="output">Returns the world-space centroid of the polyhedron.</param>
		public void Center(out Vector3 output)
		{
			output = _center;
		}

		/// <summary>
		/// Gets a single vertex in world space.
		/// </summary>
		/// <param name="index">The index of the vertex to retrieve.</param>
		/// <param name="output">Returns the world-space vertex.</param>
		public void World(int index, out Vector3 output)
		{
			output = _world[index];
		}

		/// <summary>
		/// Gets a single vertex in local space.
		/// </summary>
		/// <param name="index">The index of the vertex to retrieve.</param>
		/// <param name="output">Returns the local-space vertex.</param>
		public void Body(int index, out Vector3 output)
		{
			output = _compiled.Body[index];
		}

		/// <summary>
		/// Gets all neighbors of a given vertex. Neighbors are vertices that share a common edge.
		/// </summary>
		/// <param name="index">The index of the vertex whose neighbors are retrieved.</param>
		/// <returns>Returns an array of indices for neighboring vertices.</returns>
		public int[] Neighbors(int index)
		{
			return _compiled.Neighbors[index];
		}

		/// <summary>
		/// Gets the pair of vertices that make up a single edge.
		/// </summary>
		/// <param name="index">The index of the edge to retrieve.</param>
		/// <returns>Returns a pair of vertex indices that make up the edge.</returns>
		public int[] Edge(int index)
		{
			return _compiled.Edges[index];
		}

		/// <summary>
		/// Gets a single edge vector, indicating the direction of an edge.
		/// </summary>
		/// <param name="index">The index of the edge vector to retrieve.</param>
		/// <param name="output">Returns the vector containing the edge direction.</param>
		public void EdgeVector(int index, out Vector3 output)
		{
			output = _edgeVectors[index];
		}

		/// <summary>
		/// Gets a list of vertex indices that make up a single face.
		/// </summary>
		/// <param name="index">The index of the face to retrieve.</param>
		/// <returns>Returns an array of vertex indices that make up the face, in counter-clockwise order.</returns>
		public int[] Face(int index)
		{
			return _compiled.Faces[index];
		}

		/// <summary>
		/// Gets a single face normal, or the normal of the plane that is parallel to the face.
		/// </summary>
		/// <param name="index">The index of the face whose normal is retrieved.</param>
		/// <param name="output">Returns the face normal.</param>
		public void FaceNormal(int index, out Vector3 output)
		{
			output = _faceNormals[index];
		}

		/// <summary>
		/// Gets the vertex that is the most extreme, or furthest along, the specified direction.
		/// </summary>
		/// <param name="d">The normalized direction vector along which to search for an extreme vertex.</param>
		/// <returns>Returns the extreme vertex.</returns>
		public PolyhedronFeature ExtremeVertex(ref Vector3 d)
		{
			PolyhedronFeature e;
			Vector3 v;

			e.Type = PolyhedronFeatureType.Vertex;
			e.Index = 0;
			World(0, out v);
			Vector3.Dot(ref d, ref v, out e.X);
			bool done = false;
			int vIndex;

			while (!done)
			{
				vIndex = e.Index;
				done = true;
				for (int i = 0; i < _compiled.Neighbors[vIndex].Length; i++)
				{
					float x;
					World(_compiled.Neighbors[vIndex][i], out v);
					Vector3.Dot(ref d, ref v, out x);
					if (x - e.X >= Constants.Epsilon)
					{
						e.Index = _compiled.Neighbors[vIndex][i];
						e.X = x;
						done = false;
					}
				}
			}
			return e;
		}

		/// <summary>
		/// Gets the polyhedron feature that is the most extreme, or furthest along, the specified direction and for which
		/// all vertices are at least as extreme as the specified depth.
		/// </summary>
		/// <remarks>
		/// If two vertices are more extreme than the specified depth, then an edge is returned. If more than two vertices are
		/// more extreme than the specified depth, then a face is returned. The result will always include the vertex of maximum
		/// extremity.
		/// </remarks>
		/// <param name="d">The normalized direction vector along which to search for an extreme feature.</param>
		/// <param name="depth">All vertices that make up the returned feature must be at least this "deep" along the
		/// given direction.</param>
		/// <returns>Returns the extreme feature.</returns>
		public PolyhedronFeature ExtremeFeature(ref Vector3 d, float depth)
		{
			PolyhedronFeature e = ExtremeVertex(ref d);
			Vector3 v;

			if (depth - e.X >= Constants.Epsilon)
			{
				e.Type = PolyhedronFeatureType.None;
				return e;
			}

			int vIndex = e.Index;
			for (int i = 0; i < _compiled.Neighbors[vIndex].Length; i++)
			{
				float x;
				int newIndex = _compiled.Neighbors[vIndex][i];
				World(newIndex, out v);
				Vector3.Dot(ref d, ref v, out x);
				if (x >= depth)
				{
					if (e.Type == PolyhedronFeatureType.Vertex)
					{
						for (int j = 0; j < _compiled.Edges.Length; j++)
						{
							if ((_compiled.Edges[j][0] == vIndex && _compiled.Edges[j][1] == newIndex) ||
								(_compiled.Edges[j][0] == newIndex && _compiled.Edges[j][1] == vIndex))
							{
								e.Type = PolyhedronFeatureType.Edge;
								e.Index = j;
								break;
							}
						}
					}
					else if (e.Type == PolyhedronFeatureType.Edge)
					{
						for (int j = 0; j < _compiled.Faces.Length; j++)
						{
							int count = 0;
							for (int k = 0; k < _compiled.Faces[j].Length; k++)
							{
								if (_compiled.Faces[j][k] == _compiled.Edges[e.Index][0] ||
									_compiled.Faces[j][k] == _compiled.Edges[e.Index][1] ||
									_compiled.Faces[j][k] == newIndex) count++;
								if (count > 2) break;
							}
							if (count > 2)
							{
								e.Type = PolyhedronFeatureType.Face;
								e.Index = j;
								break;
							}
						}
					}
					else continue;
				}
			}

			return e;
		}

		/// <summary>
		/// Gets the polyhedron feature that is the most extreme, or furthest along, the specified direction.
		/// </summary>
		/// <remarks>
		/// If two vertices are equally far along the given direction, then an edge feature is returned. If more
		/// than two vertices are equally far, then the extreme feature is a face.
		/// </remarks>
		/// <param name="d">The normalized direction vector along which to search for an extreme feature.</param>
		/// <returns>Returns the extreme feature that is furthest along the specified direction.</returns>
		public PolyhedronFeature ExtremeFeature(ref Vector3 d)
		{
			PolyhedronFeature e = ExtremeVertex(ref d);
			Vector3 v;

			int vIndex = e.Index;
			for (int i = 0; i < _compiled.Neighbors[vIndex].Length; i++)
			{
				float x;
				int newIndex = _compiled.Neighbors[vIndex][i];
				v = _world[newIndex];
				Vector3.Dot(ref d, ref v, out x);
				if (FloatHelper.Equals(x, e.X))
				{
					if (e.Type == PolyhedronFeatureType.Vertex)
					{
						for (int j = 0; j < _compiled.Edges.Length; j++)
						{
							if ((_compiled.Edges[j][0] == vIndex && _compiled.Edges[j][1] == newIndex) ||
								(_compiled.Edges[j][0] == newIndex && _compiled.Edges[j][1] == vIndex))
							{
								e.Type = PolyhedronFeatureType.Edge;
								e.Index = j;
								break;
							}
						}
					}
					else if (e.Type == PolyhedronFeatureType.Edge)
					{
						for (int j = 0; j < _compiled.Faces.Length; j++)
						{
							int count = 0;
							for (int k = 0; k < _compiled.Faces[j].Length; k++)
							{
								if (_compiled.Faces[j][k] == _compiled.Edges[e.Index][0] ||
									_compiled.Faces[j][k] == _compiled.Edges[e.Index][1] ||
									_compiled.Faces[j][k] == newIndex) count++;
								if (count > 2) break;
							}
							if (count > 2)
							{
								e.Type = PolyhedronFeatureType.Face;
								e.Index = j;
								break;
							}
						}
					}
					else break;
				}
			}

			return e;
		}

		/// <summary>
		/// Apply a transform to the collision skin part to bring it into world space.
		/// </summary>
		/// <param name="transform">The world-space transform to apply.</param>
		public override void ApplyTransform(ref Transform transform)
		{
			Vector3.Transform(_compiled.Body, ref transform.Combined, _world);
			Vector3.Transform(_compiled.FaceNormals, ref transform.Orientation, _faceNormals);
			Vector3.Transform(_compiled.EdgeVectors, ref transform.Orientation, _edgeVectors);
			_center = transform.Position;
		}

		/// <summary>
		/// Retrieves the bounding box enclosing the collision skin part.
		/// </summary>
		/// <param name="aabb">Returns the bounding box for this part.</param>
		public override void BoundingBox(out AlignedBox aabb)
		{
			AlignedBox.Fit(_world, out aabb);
		}

		/// <summary>
		/// Determines whether the specified point, when projected onto the face plane of the polyhedron, is within the face.
		/// </summary>
		/// <param name="faceIndex">The index of the face in which to check containment.</param>
		/// <param name="p">The point in world-space that is either inside or outside the face.</param>
		/// <param name="inclusive">Indicates whether points on the edge or vertices of a face should be considered within the face.</param>
		/// <returns>Returns a value indicating whether the point is contained within the face.</returns>
		public bool IsPointOnFace(int faceIndex, ref Vector3 p, bool inclusive)
		{
			int[] face = Face(faceIndex);
			int low = 0, high = face.Length;
			Vector3 normal;
			FaceNormal(faceIndex, out normal);
			Segment s;
			Vector3 p0;
			World(face[0], out p0);
			do
			{
				int mid = (low + high) / 2;
				Vector3 p1;
				World(face[mid], out p1);
				s = new Segment(p0, p1);
				if ((mid == face.Length - 1 || mid == 1) &&
					s.DistanceSquaredTo(ref p) < Constants.Epsilon)
				{
					return inclusive;
				}
				if (GeometryHelper.IsTriangleCcw(ref normal, ref s.P1, ref s.P2, ref p))
				{
					low = mid;
				}
				else
				{
					high = mid;
				}
			}
			while (low + 1 < high);
			if (low == 0 || high == face.Length) return false;
			World(face[low], out s.P1);
			World(face[high], out s.P2);
			return GeometryHelper.IsTriangleCcw(ref normal, ref s.P1, ref s.P2, ref p) ||
				(inclusive && s.DistanceSquaredTo(ref p) < Constants.Epsilon);
		}

		/// <summary>
		/// Intersects a segment with this collision skin part and returns the intersection point that is nearest to the
		/// beginning of the segment.
		/// </summary>
		/// <param name="segment">The segment to intersect with.</param>
		/// <param name="scalar">Returns a value between 0 and 1 indicating where on the segment the first intersection occurs.</param>
		/// <param name="point">Returns the point of the first intersection.</param>
		/// <returns>Returns a value indicating whether the segment intersects with the part.</returns>
		public override bool Intersect(ref Segment segment, out float scalar, out Vector3 point)
		{
			scalar = float.PositiveInfinity;
			point = Vector3.Zero;
			float scalar1;
			Vector3 point1;
			for (int i = 0; i < FaceCount; i++)
			{
				var face = Face(i);
				Vector3 p0;
				World(face[0], out p0);
				var plane = new Plane(p0, _faceNormals[i]);
				if (plane.Intersect(ref segment, out scalar1, out point1) &&
					scalar1 < scalar && IsPointOnFace(i, ref point1, true))
				{
					scalar = scalar1;
					point = point1;
				}
			}
			return !float.IsPositiveInfinity(scalar);
		}
	}
}
