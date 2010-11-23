#pragma warning disable 0420

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using Microsoft.Xna.Framework;

namespace Henge3D
{
	#region TaskException class

	public class TaskException : Exception
	{
		Action<TaskParams> _task;

		internal Action<TaskParams> Task { get { return _task; } }

		internal TaskException(Action<TaskParams> task, Exception e)
			: base("Unhandled exception in task ("
			+ task.Target.ToString() + ", " + task.Method.ToString() + "): " + e.Message, e)
		{
			_task = task;
		}
	}

	#endregion

	#region TaskManagerException class

	public class TaskManagerException : Exception
	{
		TaskException[] _exceptions;

		public TaskException[] Exceptions { get { return _exceptions; } }

		public TaskManagerException(TaskException[] exceptions)
		{
			_exceptions = exceptions;
		}
	}

	#endregion

	#region TaskParams struct

	public struct TaskParams
	{
		public object Param1;
		public object Param2;

		public TaskParams(object param1, object param2)
		{
			Param1 = param1;
			Param2 = param2;
		}
	}

	#endregion

	/// <summary>
	/// Facilitates the concurrent execution of tasks.
	/// </summary>
	public class TaskManager : IDisposable
	{
		private static readonly int[] _xBoxCoreMap = new int[] { 1, 3, 4, 5 };
		private static TaskManager _current;
		private static int _threadCount;
		private static bool _isThreadingEnabled = true;

		static TaskManager()
		{
#if WINDOWS
			_threadCount = System.Environment.ProcessorCount;
#elif XBOX
			_threadCount = _xBoxCoreMap.Length;
#else
			_threadCount = 1;
#endif
		}

		/// <summary>
		/// Gets a reference to the task manager currently managing tasks in the process.
		/// </summary>
		public static TaskManager Current
		{
			get
			{
				if (_current == null)
					_current = new TaskManager();
				return _current;
			}
		}

		/// <summary>
		/// Gets or sets a value indicating whether multi-threading is enabled. If this value is set to false, all tasks will be
		/// executed in serial on a single processor.
		/// </summary>
		public static bool IsThreadingEnabled { get { return _isThreadingEnabled; } set { _isThreadingEnabled = value; } }

		/// <summary>
		/// Gets the supproted number of threads.
		/// </summary>
		public static int ThreadCount { get { return _threadCount; } }

		/// <summary>
		/// Gets the index of the calling thread.
		/// </summary>
		public static int CurrentThreadIndex
		{
			get
			{
				for (int i = 0; i < _threadCount; i++)
				{
					if (_current._threads[i].ManagedThreadId == Thread.CurrentThread.ManagedThreadId)
					{
						return i;
					}
				}
				return -1;
			}
		}

		private volatile bool _disposing = false, _running = false;
		private volatile int _waitingThreadCount = 0, _currentTaskIndex = 0;
		private volatile List<Action<TaskParams>> _tasks;
		private volatile List<TaskParams> _params;
		private Thread[] _threads;

		private object _exceptionsLock;
		private List<TaskException> _exceptions;
		private AutoResetEvent _taskInitWaitHandle;
		private ManualResetEvent _managerWaitHandleA, _managerWaitHandleB, _managerCurrentWaitHandle;

		private TaskManager()
		{
			_tasks = new List<Action<TaskParams>>();
			_params = new List<TaskParams>();
			_exceptions = new List<TaskException>();
			_exceptionsLock = new object();
			_threads = new Thread[_threadCount];
			_taskInitWaitHandle = new AutoResetEvent(false);
			_managerWaitHandleA = new ManualResetEvent(false);
			_managerWaitHandleB = new ManualResetEvent(false);
			_managerCurrentWaitHandle = _managerWaitHandleA;

			_threads[0] = Thread.CurrentThread;

#if XBOX
			Thread.CurrentThread.SetProcessorAffinity(_xBoxCoreMap[0]);
#endif

			for (int i = 1; i < _threadCount; i++)
			{
				_threads[i] = new Thread(() =>
					{
#if XBOX
						Thread.CurrentThread.SetProcessorAffinity(_xBoxCoreMap[i]);
#endif
						_taskInitWaitHandle.Set();
						ThreadProc();
					});
				_threads[i].Start();
				_taskInitWaitHandle.WaitOne();
			}
		}

		/// <summary>
		/// Add a new task to the task manager to be executed at a later time.
		/// </summary>
		/// <param name="task">The task to add.</param>
		/// <param name="parameters">Optinoal parameters to supply to the task.</param>
		public void AddTask(Action<TaskParams> task, TaskParams parameters)
		{
			_tasks.Add(task);
			_params.Add(parameters);
		}

		/// <summary>
		/// Add a new task to the task manager to be executed at a later time.
		/// </summary>
		/// <param name="task">The task to add.</param>
		/// <param name="param1">The first optional parameter.</param>
		/// <param name="param2">The second optional parameter.</param>
		public void AddTask(Action<TaskParams> task, object param1, object param2)
		{
			this.AddTask(task, new TaskParams(param1, param2));
		}

		/// <summary>
		/// Add a new task to the task manager to be executed at a later time.
		/// </summary>
		/// <param name="task">The task to add.</param>
		/// <param name="param1">An optional parameter to supply to the task.</param>
		public void AddTask(Action<TaskParams> task, object param)
		{
			this.AddTask(task, new TaskParams(param, null));
		}

		/// <summary>
		/// Add a new task to the task manager to be executed at a later time.
		/// </summary>
		public void AddTask(Action<TaskParams> task)
		{
			this.AddTask(task, new TaskParams(null, null));
		}

		/// <summary>
		/// Executes all currently queued tasks. This method will not return until every task has completed.
		/// </summary>
		public void Execute()
		{
			if (_disposing) throw new InvalidOperationException();
			if (_running) throw new InvalidOperationException();

			if (_tasks.Count < 1) return;
			_running = true;

			try
			{
				if (!_isThreadingEnabled)
				{
					for (int i = 0; i < _tasks.Count; i++)
						_tasks[i](_params[i]);
				}
				else
				{
					_currentTaskIndex = 0;
					_waitingThreadCount = 0;
					_managerCurrentWaitHandle.Set();

					TaskPump();

					while (_waitingThreadCount < _threadCount - 1)
						Thread.Sleep(0);

					if (_exceptions.Count > 0)
					{
						var e = new TaskManagerException(_exceptions.ToArray());
						_exceptions.Clear();
						throw e;
					}
				}
			}
			finally
			{
				_managerCurrentWaitHandle.Reset();
				_managerCurrentWaitHandle = _managerCurrentWaitHandle == _managerWaitHandleA
					? _managerWaitHandleB : _managerWaitHandleA;
				_tasks.Clear();
				_params.Clear();
				_running = false;
			}
		}

		/// <summary>
		/// Dispose the task manager.
		/// </summary>
		public void Dispose()
		{
			_disposing = true;
            if (_running)
            {
                _managerWaitHandleA.Set();
                _managerWaitHandleB.Set();
            }
		}

		private void ThreadProc()
		{
			while (true)
			{
				Interlocked.Increment(ref _waitingThreadCount);
				_managerWaitHandleA.WaitOne();

				if (_disposing)
					return;
				else
					TaskPump();

				Interlocked.Increment(ref _waitingThreadCount);
				_managerWaitHandleB.WaitOne();

				if (_disposing)
					return;
				else
					TaskPump();
			}
		}

		private void TaskPump()
		{
			int count = _tasks.Count;

			while (_currentTaskIndex < count)
			{
				int taskIndex = _currentTaskIndex;

				if (taskIndex == Interlocked.CompareExchange(ref _currentTaskIndex, taskIndex + 1, taskIndex)
					&& taskIndex < count)
				{
					try
					{
						_tasks[taskIndex](_params[taskIndex]);
					}
					catch (Exception e)
					{
						lock (_exceptionsLock)
						{
							_exceptions.Add(new TaskException(_tasks[taskIndex], e));
						}
					}
				}
			}
		}
	}
}
