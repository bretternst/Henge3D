using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;

namespace Henge3D.Holodeck
{
	public interface IViewManager
	{
		GraphicsDevice Device { get; } 
		Vector3 UpAxis { get; set; }
		Vector3 ForwardAxis { get; set; }
		Vector3 Position { get; set; }
		Vector3 Direction { get; set; }
		Matrix Projection { get; set; }
		Matrix View { get; }
		float MaxPitch { get; set; }
		float MinPitch { get; set; }
		float Pitch { get; set; }
		float Yaw { get; set; }
		void SetProjection(float viewPlaneNear, float viewPlaneFar, float fieldOfView);
		void SetProjection(float viewPlaneFar);
		void Move(Vector3 delta);
	}
}
