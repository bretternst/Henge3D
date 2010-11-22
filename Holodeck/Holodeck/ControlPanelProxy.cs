using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows.Threading;
using Microsoft.Xna.Framework;

namespace Henge3D.Holodeck
{
	class ControlPanelProxy
	{
		private ControlPanel _panel;
		private Dispatcher _disp;

		public ControlPanelProxy(ControlPanel p)
		{
			_panel = p;
			_disp = p.Dispatcher;
		}

		public string SelectedAsset
		{
			get
			{
				return _disp.Invoke((Func<string>)(() => _panel.SelectedAsset)) as string;
			}
		}

		public float SpawnVelocity
		{
			get
			{
				return _panel.SpawnVelocity;
			}
		}

		public Vector3 SpawnAngularVelocity
		{
			get
			{
				return _panel.SpawnAngularVelocity;
			}
		}

		public bool IsPaused
		{
			get
			{
				return _panel.IsPaused;
			}
		}

		public void SetFps(int fps)
		{
			_disp.BeginInvoke((Action)(() => _panel.SetFps(fps)));
		}

		public void SetModelList(string[] assetNames)
		{
			_disp.BeginInvoke((Action)(() => _panel.SetModelList(assetNames)));
		}

		public void Exit()
		{
			_disp.InvokeShutdown();
		}
	}
}
