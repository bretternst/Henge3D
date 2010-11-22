using System;
using System.Threading;
#if WINDOWS
using System.Windows;
using System.Windows.Threading;
using System.Windows.Interop;
#endif

namespace Henge3D.Holodeck
{
	static class Program
	{
#if WINDOWS
		private static Dispatcher _ctlDisp;
		private static ControlPanelProxy _proxy;
		private static ManualResetEvent _ready = new ManualResetEvent(false);

		public static ControlPanelProxy Proxy { get { return _proxy; } }
		public static Dispatcher Dispatcher { get { return _ctlDisp; } }

		private static void DoControlPanel(object o)
		{
			_ctlDisp = Dispatcher.CurrentDispatcher;
			Application ctlApp = new Application();

			ControlPanel panel = new ControlPanel((IntPtr)o);
			_proxy = new ControlPanelProxy(panel);

			_ready.Set(); // go!
			ctlApp.Run(panel);
		}

		private static void Main(string[] args)
		{
			using (Holodeck game = new Holodeck())
			{
				Thread ctlThread = new Thread(new ParameterizedThreadStart(DoControlPanel));
				ctlThread.SetApartmentState(ApartmentState.STA);
				ctlThread.Start(game.Window.Handle);

				_ready.WaitOne();

				game.Run();
				_proxy.Exit();
			}
		}
#else
		static void Main(string[] args)
		{
			using (Holodeck game = new Holodeck())
			{
				game.Run();
			}
		}
#endif
	}
}
