using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Runtime.InteropServices;
using System.Windows.Threading;

namespace Henge3D.Holodeck
{
	public delegate int HookProc(int nCode, IntPtr wParam, IntPtr lParam);

	[StructLayout(LayoutKind.Sequential)]
	struct WindowRect
	{
		public WindowRect(int x1, int y1, int x2, int y2)
		{
			X1 = x1;
			Y1 = y1;
			X2 = x2;
			Y2 = y2;
		}

		public int X1;
		public int Y1;
		public int X2;
		public int Y2;
	}

	[StructLayout(LayoutKind.Sequential)]
	public struct CWPRETSTRUCT
	{
		public IntPtr lResult;
		public IntPtr lParam;
		public IntPtr wParam;
		public uint message;
		public IntPtr hwnd;
	};
	
	static class Interop
	{
		[DllImport("user32.dll")]
		public static extern IntPtr SetWindowsHookEx(int idHook, HookProc lpfn, IntPtr hInstance, int threadId);

		[DllImport("user32.dll")]
		public static extern bool UnhookWindowsHookEx(int idHook);

		[DllImport("user32.dll")]
		public static extern int CallNextHookEx(IntPtr idHook, int nCode, IntPtr wParam, IntPtr lParam);

		[DllImport("user32.dll")]
		static extern bool GetWindowRect(IntPtr hWnd, out WindowRect lpRect);

		[DllImport("User32.dll")]
		static extern bool MoveWindow(IntPtr handle, int x, int y, int width, int height, bool redraw);

		[DllImport("user32.dll")]
		static extern int GetWindowThreadProcessId(IntPtr hWnd, IntPtr ProcessId);

		[DllImport("user32.dll")]
		static extern IntPtr GetWindowLong(IntPtr hWnd, int nIndex);

		[DllImport("user32.dll")]
		static extern void SetWindowLong(IntPtr hWnd, int nIndex, IntPtr dwNewLong);

		[DllImport("user32.dll")]
		static public extern bool IsWindow(IntPtr hWnd);

		[DllImport("user32.dll")]
		static extern bool SetActiveWindow(IntPtr handle);

		[DllImport("user32.dll")]
		static public extern bool SetForegroundWindow(IntPtr handle);

		[DllImport("user32.dll")]
		static public extern bool ShowWindow(IntPtr handle);

		[DllImport("user32.dll")]
		static public extern bool IsIconic(IntPtr handle);

		static public void FocusWindow(IntPtr handle)
		{
			SetForegroundWindow(handle);
			SetActiveWindow(handle);
		}
		
		static public WindowRect GetWindowBounds(IntPtr handle)
		{
			WindowRect r;
			if (!GetWindowRect(handle, out r))
				throw new InvalidOperationException();
			return r;
		}

		static public void SetWindowBounds(IntPtr handle, WindowRect rect)
		{
			if (!MoveWindow(handle, rect.X1, rect.Y1, rect.X2 - rect.X1, rect.Y2 - rect.Y1, true))
				throw new InvalidOperationException();
		}

		static int GetWindowThreadId(IntPtr handle)
		{
			return GetWindowThreadProcessId(handle, IntPtr.Zero);
		}

		static IntPtr GetWindowInstance(IntPtr handle)
		{
			return GetWindowLong(handle, -6);
		}

		static public void SetToolWindowStyle(IntPtr handle)
		{
			int exStyle = (int)GetWindowLong(handle, -20);
			exStyle |= 0x80;
			SetWindowLong(handle, -20, (IntPtr)exStyle);
		}

		static List<HookProc> hookProcs = new List<HookProc>();

		static public void SetWindowListener(IntPtr handle, int[] messages, Dispatcher disp, Action<uint> callback)
		{
			int threadId = Interop.GetWindowThreadId(handle);
			IntPtr hInstance = Interop.GetWindowInstance(handle);
			IntPtr hHook = IntPtr.Zero; 
			
			HookProc proc = (int nCode, IntPtr wParam, IntPtr lParam) =>
			{
				CWPRETSTRUCT msg = (CWPRETSTRUCT)System.Runtime.InteropServices.Marshal.PtrToStructure(lParam, typeof(CWPRETSTRUCT));
				if (nCode >= 0)
				{
					if (msg.message == 2)
					{
						Interop.UnhookWindowsHookEx(12);
					}
					else if (messages.Contains((int)msg.message))
					{
						disp.BeginInvoke(callback, msg.message);
					}
				}
				return Interop.CallNextHookEx(hHook, nCode, wParam, lParam);
			};
			hookProcs.Add(proc);

			hHook = Interop.SetWindowsHookEx(12, proc, hInstance, threadId);
		}
	}
}
