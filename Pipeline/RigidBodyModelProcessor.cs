using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;
using System.ComponentModel;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Content.Pipeline;
using Microsoft.Xna.Framework.Content.Pipeline.Graphics;
using Microsoft.Xna.Framework.Content.Pipeline.Processors;
using Henge3D.Physics;

namespace Henge3D.Pipeline
{
	[ContentProcessor(DisplayName="Rigid Body Model Processor")]
	public class RigidBodyModelProcessor : ModelProcessor
	{
		const string TYPE_ATTR_NAME = "type";
		const string ELASTICITY_ATTR_NAME = "elasticity";
		const string ROUGHNESS_ATTR_NAME = "roughness";
		const string DENSITY_ATTR_NAME = "density";
		const string SHAPE_ATTR_NAME = "shape";

		private float _defaultDensity = 1f, _defaultElasticity = 0f, _defaultRoughness = 0.5f;

		[DefaultValue(1f)]
		[DisplayName("Default Density")]
		[Description("The default density of all meshes, used when calculating mass properties.")]
		public float DefaultDensity { get { return _defaultDensity; } set { _defaultDensity = value; } }

		[DefaultValue(0f)]
		[DisplayName("Default Elasticity")]
		[Description("The default elasticity of all meshes, used when calculating collision response.")]
		public float DefaultElasticity { get { return _defaultElasticity; } set { _defaultElasticity = value; } }

		[DefaultValue(0.5f)]
		[DisplayName("Default Roughness")]
		[Description("The default roughness of all meshes, used when calculating friction.")]
		public float DefaultRoughness { get { return _defaultRoughness; } set { _defaultRoughness = value; } }

		public override ModelContent Process(NodeContent input, ContentProcessorContext context)
		{
			var attributes = input.Children.ToDictionary(n => n.Name, n => n.OpaqueData);

			var nodesToRemove = (from node in input.Children
								 where node.OpaqueData.GetAttribute(TYPE_ATTR_NAME, MeshType.Both) == MeshType.Physical
								 select node).ToArray();

			ModelContent model = base.Process(input, context);
			var parts = new List<CompiledPart>();
			var materials = new List<Material>();
			var mass = new MassProperties();
			var centerOfMass = Vector3.Zero;

			foreach (var mesh in model.Meshes)
			{
				MeshType type = MeshType.Both;
				PhysicalShape shape = PhysicalShape.Mesh;
				float elasticity = _defaultElasticity, roughness = _defaultRoughness, density = _defaultDensity;

				if (attributes.ContainsKey(mesh.Name))
				{
					type = attributes[mesh.Name].GetAttribute(TYPE_ATTR_NAME, MeshType.Both);
					if (type == MeshType.Visual) continue;
					elasticity = attributes[mesh.Name].GetAttribute(ELASTICITY_ATTR_NAME, _defaultElasticity);
					roughness = attributes[mesh.Name].GetAttribute(ROUGHNESS_ATTR_NAME, _defaultRoughness);
					density = attributes[mesh.Name].GetAttribute(DENSITY_ATTR_NAME, _defaultDensity);
					shape = attributes[mesh.Name].GetAttribute(SHAPE_ATTR_NAME, PhysicalShape.Mesh);
				}

				var meshCenterOfMass = Vector3.Zero;
				var meshMass = MassProperties.Immovable;
				CompiledPart meshPart = null;

				int[] indices = mesh.IndexBuffer.ToArray();
				Vector3[] vertices = MeshToVertexArray(context.TargetPlatform, mesh);

				switch (shape)
				{
					case PhysicalShape.Mesh:
						{
							meshPart = new CompiledMesh(vertices, indices);
							meshMass = MassProperties.Immovable;
							meshCenterOfMass = GetMeshTranslation(mesh);
						}
						break;
					case PhysicalShape.Polyhedron:
						{
							var hull = new ConvexHull3D(vertices);
							meshPart = hull.ToPolyhedron();
							meshMass = MassProperties.FromTriMesh(density, vertices, indices, out meshCenterOfMass);
						}
						break;
					case PhysicalShape.Sphere:
						{
							Sphere s;
							Sphere.Fit(vertices, out s);
							meshPart = new CompiledSphere(s.Center, s.Radius);
							meshMass = MassProperties.FromSphere(density, s.Center, s.Radius);
							meshCenterOfMass = s.Center;
						}
						break;
					case PhysicalShape.Capsule:
						{
							Capsule c;
							Capsule.Fit(vertices, out c);
							meshPart = new CompiledCapsule(c.P1, c.P2, c.Radius);
							meshMass = MassProperties.FromCapsule(density, c.P1, c.P2, c.Radius, out meshCenterOfMass);
						}
						break;
				}
				parts.Add(meshPart);
				materials.Add(new Material(elasticity, roughness));
				Vector3.Multiply(ref meshCenterOfMass, meshMass.Mass, out meshCenterOfMass);
				Vector3.Add(ref centerOfMass, ref meshCenterOfMass, out centerOfMass);
				mass.Mass += meshMass.Mass;
				meshMass.Inertia.M44 = 0f;
				Matrix.Add(ref mass.Inertia, ref meshMass.Inertia, out mass.Inertia);
			}

			// compute mass properties
			Vector3.Divide(ref centerOfMass, mass.Mass, out centerOfMass);
			mass.Inertia.M44 = 1f;
			MassProperties.TranslateInertiaTensor(ref mass.Inertia, -mass.Mass, centerOfMass, out mass.Inertia);
			if (centerOfMass.Length() >= Constants.Epsilon)
			{
				var transform = Matrix.CreateTranslation(-centerOfMass.X, -centerOfMass.Y, -centerOfMass.Z);
				foreach (var p in parts)
				{
					p.Transform(ref transform);
				}

				transform = model.Root.Transform;
				transform.M41 -= centerOfMass.X;
				transform.M42 -= centerOfMass.Y;
				transform.M43 -= centerOfMass.Z;
				model.Root.Transform = transform;
			}

			mass = new MassProperties(mass.Mass, mass.Inertia);
			var rbm = new RigidBodyModel(mass, parts.ToArray(), materials.ToArray());

			// remove non-visual nodes
			if (nodesToRemove.Length > 0)
			{
				foreach (var node in nodesToRemove)
					input.Children.Remove(node);
				model = base.Process(input, context);
			}

			model.Tag = rbm;
			return model;
		}

		private float GetMaxDistance(Vector3 center, IList<Vector3> vertices)
		{
			float maxDist = 0f;
			for (int i = 0; i < vertices.Count; i++)
			{
				var p = vertices[i];
				float dist;
				Vector3.DistanceSquared(ref center, ref p, out dist);
				if (dist > maxDist)
					maxDist = dist;
			}
			return (float)Math.Sqrt(maxDist);
		}

		private Vector3 GetMeshTranslation(ModelMeshContent mesh)
		{
			var pos = Vector3.Zero;
			var bone = mesh.ParentBone;
			while (bone != null)
			{
				pos += bone.Transform.Translation;
				bone = bone.Parent;
			}
			return pos;
		}

		private Vector3[] MeshToVertexArray(TargetPlatform platform, ModelMeshContent mesh)
		{
			MemoryStream ms;
			if (platform == TargetPlatform.Xbox360)
			{
				ms = new MemoryStream(ReverseByteOrder(mesh.VertexBuffer.VertexData));
			}
			else
			{
				ms = new MemoryStream(mesh.VertexBuffer.VertexData);
			}
			BinaryReader reader = new BinaryReader(ms);

			var elems = mesh.MeshParts[0].GetVertexDeclaration();
			int count = mesh.VertexBuffer.VertexData.Length /
				VertexDeclaration.GetVertexStrideSize(elems, 0);
			var vertices = new Vector3[count];
			for (int i = 0; i < count; i++)
			{
				foreach (var elType in elems)
				{
					if (elType.VertexElementUsage == VertexElementUsage.Position)
					{
						vertices[i].X = reader.ReadSingle();
						vertices[i].Y = reader.ReadSingle();
						vertices[i].Z = reader.ReadSingle();
					}
					else
					{
						switch (elType.VertexElementFormat)
						{
							case VertexElementFormat.Color:
								reader.ReadUInt32();
								break;
							case VertexElementFormat.Vector2:
								reader.ReadSingle();
								reader.ReadSingle();
								break;
							case VertexElementFormat.Vector3:
								reader.ReadSingle();
								reader.ReadSingle();
								reader.ReadSingle();
								break;
							default:
								throw new InvalidContentException("Unrecognized element type in vertex buffer: " + elType.ToString());
						}
					}
				}
			}

			var transforms = new Stack<Matrix>();
			var bone = mesh.ParentBone;
			while(bone != null)
			{
				transforms.Push(bone.Transform);
				bone = bone.Parent;
			}

			var transform = Matrix.Identity;
			while (transforms.Count > 0)
			{
				transform *= transforms.Pop();
			}

			Vector3.Transform(vertices, ref transform, vertices);
			return vertices;
		}

		private byte[] ReverseByteOrder(byte[] source)
		{
			byte[] dest = new byte[source.Length];
			for (int i = 0; i < source.Length; i += 4)
			{
				dest[i] = source[i + 3];
				dest[i + 1] = source[i + 2];
				dest[i + 2] = source[i + 1];
				dest[i + 3] = source[i];
			}
			return dest;
		}
	}
}
