using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Shapes;
using System.Windows.Interop;
using Microsoft.Xna.Framework;

using Henge3D;

namespace Henge3D.Holodeck
{
	/// <summary>
	/// Interaction logic for ControlPanel.xaml
	/// </summary>
	public partial class ControlPanel : Window
	{
		IntPtr masterHandle;
		IntPtr thisHandle;

		public ControlPanel(IntPtr handle)
		{
			masterHandle = handle;

			InitializeComponent();

			this.Loaded += new RoutedEventHandler(ControlPanel_Loaded);
			this.WindowState = WindowState.Minimized;
		}

		#region Proxied functions

		public string SelectedAsset
		{
			get
			{
				return templateList.SelectedValue as string;
			}
		}

		public float SpawnVelocity { get; set; }
		public float SpawnAngularX { get; set; }
		public float SpawnAngularY { get; set; }
		public float SpawnAngularZ { get; set; }
		public Vector3 SpawnAngularVelocity
		{
			get
			{
				return new Vector3(SpawnAngularX, 0, 0) +
					new Vector3(0, SpawnAngularY, 0) +
					new Vector3(0, 0, SpawnAngularZ);
			}
		}

		bool isPaused = false;
		public bool IsPaused { get { return isPaused; } set { isPaused = value; } }
		public void SetFps(int fps)
		{
			lblFps.Content = fps.ToString();
		}
		public void SetModelList(string[] assetNames)
		{
			templateList.ItemsSource = assetNames;
			templateList.SelectedIndex = 0;
		}

		#endregion

		#region Window cling machinery

		void ControlPanel_Loaded(object sender, RoutedEventArgs e)
		{
			thisHandle = (new WindowInteropHelper(this)).Handle;

			Interop.SetWindowListener(masterHandle, new int[] { 0x5, 0x06, 0x216, 0x07, 0x08 }, Dispatcher, ClingToMaster);
			Interop.SetToolWindowStyle(thisHandle);
			Interop.FocusWindow(masterHandle);
			this.ClingToMaster(0x05);
		}

		void ClingToMaster(uint msg)
		{
			if (Interop.IsWindow(masterHandle))
			{
				try
				{
					switch (msg)
					{
						case 0x07:
							this.Topmost = true;
							this.Topmost = false;
							Interop.SetForegroundWindow(masterHandle);
							break;
						case 0x08:
							break;
						default:
							if (Interop.IsIconic(masterHandle))
								this.WindowState = WindowState.Minimized;
							else
							{
								this.WindowState = WindowState.Normal;
								WindowRect r = Interop.GetWindowBounds(masterHandle);
								Interop.SetWindowBounds(thisHandle, new WindowRect(r.X2, r.Y1, r.X2 + (int)this.Width, r.Y2));
								this.Opacity = 1.0f;
							}
							break;
					}
				}
				catch
				{
					Dispatcher.InvokeShutdown();
				}
			}
		}

		#endregion

		private void btnReset_Click(object sender, RoutedEventArgs e)
		{
			slVelocity.Value = 0;
			slAngularX.Value = 0;
			slAngularY.Value = 0;
			slAngularZ.Value = 0;
		}
	}
}
