using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Content;

namespace Henge3D.Pipeline
{
	public class CompiledCapsule : CompiledPart
	{
		[ContentSerializer]
		private Capsule _capsule;

		public CompiledCapsule()
		{
		}

		public CompiledCapsule(Vector3 p1, Vector3 p2, float radius)
		{
			_capsule = new Capsule(p1, p2, radius);
		}

		public override Part ToCompositionPart()
		{
			return new CapsulePart(_capsule);
		}

		public override void Transform(ref Matrix transform)
		{
			Vector3.Transform(ref _capsule.P1, ref transform, out _capsule.P1);
			Vector3.Transform(ref _capsule.P2, ref transform, out _capsule.P2);
		}
	}
}
