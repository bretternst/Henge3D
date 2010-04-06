using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace Henge3D
{
	public abstract class CompiledPart
	{
		public abstract Part ToCompositionPart();
		public abstract void Transform(ref Matrix transform);
	}
}
