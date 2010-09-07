using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Content;

namespace Henge3D.Pipeline
{
	internal abstract class TriangleFunctor : TreeFunctor<Triangle, ushort>
	{
		public const int BufferSize = 1024;
		public AlignedBox BoundingBox;

		public TriangleFunctor()
			: base(BufferSize, ushort.MaxValue)
		{
		}
	}

	public class CompiledMesh : CompiledPart
	{
		private class TempNode
		{
			public int[] Children = new int[8];
			public List<int> Triangles = new List<int>();
			public AlignedBox BoundingBox;
		}

		public struct Node
		{
			public ushort[] Children;
			public ushort[] Triangles;
			public AlignedBox BoundingBox;
		}

		[ContentSerializer]
		private Vector3[] _body;

		[ContentSerializer]
		private int[][] _triangles;

		[ContentSerializer]
		private Vector3[] _normals;

		[ContentSerializer]
		private AlignedBox[] _triangleBoxes;

		[ContentSerializer]
		private Node[] _nodes;

		public Vector3[] Body { get { return _body; } }
		public int[][] Triangles { get { return _triangles; } }
		public Node[] Nodes { get { return _nodes; } }

		public CompiledMesh()
		{
		}

		public CompiledMesh(Vector3[] vertices, int[] indices)
		{
			_body = vertices;
			if (indices.Length % 3 != 0)
				throw new ArgumentException("Length of index list is not a multiple of 3.");
			if (indices.Length / 3 > ushort.MaxValue)
				throw new ArgumentException("Too many triangles in mesh.");

			_triangles = new int[indices.Length / 3][];
			_normals = new Vector3[_triangles.Length];
			int j = 0;
			for (int i = 0; i < indices.Length; i += 3)
			{
				_triangles[j] = new int[] { indices[i], indices[i + 1], indices[i + 2] };
				Vector3 v12, v13;
				Vector3.Subtract(ref _body[indices[i + 1]], ref _body[indices[i]], out v12);
				Vector3.Subtract(ref _body[indices[i + 2]], ref _body[indices[i]], out v13);
				Vector3.Cross(ref v12, ref v13, out _normals[j]);
				_normals[j++].Normalize();
			}

			BuildOctree();
		}

		public override Part ToCompositionPart()
		{
			return new MeshPart(this);
		}

		public override void Transform(ref Matrix transform)
		{
			Vector3.Transform(_body, ref transform, _body);
			BuildOctree();
		}

		internal void ProcessTriangles(TriangleFunctor tf)
		{
			Triangle tri;
			int stackIdx = 0, stackSize = tf.Stack.Length;
			int bufIdx = 0, bufSize = tf.Buffer.Length;

			var box = tf.BoundingBox;
			if (AlignedBox.Intersect(ref box, ref _nodes[0].BoundingBox) == BoxIntersectType.None) return;

			tf.Stack[0] = 0;
			while (stackIdx >= 0)
			{
				int nodeIdx = tf.Stack[stackIdx];
				if (AlignedBox.Intersect(ref box, ref _nodes[nodeIdx].BoundingBox) != BoxIntersectType.None)
				{
					for (int i = 0; i < _nodes[nodeIdx].Triangles.Length; i++)
					{
						if (AlignedBox.Intersect(ref box, ref _triangleBoxes[_nodes[nodeIdx].Triangles[i]]) != BoxIntersectType.None)
						{
							int[] vertices = _triangles[_nodes[nodeIdx].Triangles[i]];
							tri.V1 = _body[vertices[0]];
							tri.V2 = _body[vertices[1]];
							tri.V3 = _body[vertices[2]];
							tri.Normal = _normals[_nodes[nodeIdx].Triangles[i]];
							tf.Buffer[bufIdx++] = tri;
							if (bufIdx == bufSize)
							{
								tf.Process(bufSize);
								bufIdx = 0;
							}
						}
					}
				}
				stackIdx--;
				for (int i = 0; i < _nodes[nodeIdx].Children.Length; i++)
				{
					if (stackIdx == stackSize - 2)
						stackSize = tf.GrowStack();
					tf.Stack[++stackIdx] = _nodes[nodeIdx].Children[i];
				}
			}
			if (bufIdx > 0)
				tf.Process(bufIdx);
		}

		private void BuildOctree()
		{
			var tempNodes = new List<TempNode>();
			var root = new TempNode();
			tempNodes.Add(root);

			// create bounding boxes for all triangles
			_triangleBoxes = new AlignedBox[_triangles.Length];
			for (int i = 0; i < _triangles.Length; i++)
			{
				AlignedBox.Fit(ref _body[_triangles[i][0]], ref _body[_triangles[i][1]], ref _body[_triangles[i][2]], out _triangleBoxes[i]);
				AlignedBox.Merge(ref root.BoundingBox, ref _triangleBoxes[i], out root.BoundingBox);
			}

			var subBoxes = new AlignedBox[8];

			// add each triangle to the bounding box
			for (int i = 0; i < _triangles.Length; i++)
			{
				int idx = 0;
				var box = root.BoundingBox;

				while (AlignedBox.Intersect(ref tempNodes[idx].BoundingBox, ref _triangleBoxes[i]) == BoxIntersectType.AContainsB)
				{
					int sector = -1;
					for (int j = 0; j < 8; j++)
					{
						CreateChildBox(ref tempNodes[idx].BoundingBox, j, out subBoxes[j]);
						if (AlignedBox.Intersect(ref subBoxes[j], ref _triangleBoxes[i]) == BoxIntersectType.AContainsB)
						{
							sector = j;
							break;
						}
					}
					if (sector == -1)
					{
						tempNodes[idx].Triangles.Add(i);
						break;
					}
					else
					{
						if (tempNodes[idx].Children[sector] > 0)
						{
							idx = tempNodes[idx].Children[sector];
						}
						else
						{
							var child = new TempNode();
							child.BoundingBox = subBoxes[sector];
							tempNodes.Add(child);

							idx = tempNodes[idx].Children[sector] = tempNodes.Count - 1;
						}
					}
				}
			}

			_nodes = (from node in tempNodes
					  select new Node()
					  {
						  Children = (from i in node.Children where i > 0 select (ushort)i).ToArray(),
						  Triangles = (from i in node.Triangles select (ushort)i).ToArray(),
						  BoundingBox = node.BoundingBox
					  }).ToArray();
		}

		private void CreateChildBox(ref AlignedBox box, int sector, out AlignedBox output)
		{
			Vector3 size;
			Vector3.Subtract(ref box.Maximum, ref box.Minimum, out size);
			Vector3.Multiply(ref size, 0.5f, out size);

			Vector3 p;

			switch (sector % 4)
			{
				case 0: p = new Vector3(0f, 0f, 0f); break;
				case 1: p = new Vector3(0f, 1f, 0f); break;
				case 2: p = new Vector3(1f, 1f, 0f); break;
				case 3: p = new Vector3(1f, 0f, 0f); break;
				default: throw new Exception("Internal error: invalid bounding box section.");
			}
			if (sector > 3) p.Z = 1f;

			Vector3.Multiply(ref p, ref size, out output.Minimum);
			Vector3.Add(ref output.Minimum, ref box.Minimum, out output.Minimum);
			Vector3.Add(ref output.Minimum, ref size, out output.Maximum);
		}
	}
}
