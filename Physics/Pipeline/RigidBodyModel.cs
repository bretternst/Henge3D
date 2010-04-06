using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework.Content;
using Henge3D.Physics;

namespace Henge3D.Pipeline
{
	public class RigidBodyModel
	{
		[ContentSerializer]
		private MassProperties _mass;
		[ContentSerializer]
		private CompiledPart[] _parts;
		[ContentSerializer]
		private Material[] _materials;

		public MassProperties MassProperties { get { return _mass; } }
		public CompiledPart[] Parts { get { return _parts; } }
		public Material[] Materials { get { return _materials; } }

		private RigidBodyModel()
		{
		}

		public RigidBodyModel(MassProperties mass, CompiledPart[] parts,
			Material[] materials)
		{
			_mass = mass;
			_parts = parts;
			_materials = materials;

			if (_parts.Length != _materials.Length)
			{
				throw new ArgumentException("The count of supplied mesh parts and materials do not match.");
			}
		}
	}
}
