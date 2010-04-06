using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace Henge3D
{
	internal class Pool<T> : IAllocator<T> where T : class, new()
	{
		const int DefaultCapacity = 1024;
		const int DefaultGrowFactor = 2;

		private Stack<T> _free;
		private int _growFactor = DefaultGrowFactor;
		private int _nextAllocationSize = 0;

		public Pool()
		{
			_free = new Stack<T>(DefaultCapacity);
			PreAllocate(DefaultCapacity);
		}

		public Pool(int capacity, int growFactor)
		{
			_free = new Stack<T>(capacity);
			_nextAllocationSize = capacity;
			_growFactor = growFactor;

			if (capacity < 1)
				throw new ArgumentException("Capacity must be at least 1.");
		}

		protected virtual T Create()
		{
			return new T();
		}

		private void PreAllocate(int capacity)
		{
			for (int i = 0; i < capacity; i++)
			{
				_free.Push(this.Create());
			}
		}

		public T Allocate()
		{
			if (_free.Count == 0)
			{
				PreAllocate(_nextAllocationSize);
				_nextAllocationSize *= 2;
			}
			return _free.Pop();
		}

		public void Recycle(T obj)
		{
			var r = obj as IRecyclable;
			if (r != null)
				r.Recycle();

			_free.Push(obj);
		}

		public void Recycle(IList<T> objList)
		{
			for (int i = 0; i < objList.Count; i++)
			{
				var r = objList[i] as IRecyclable;
				if (r != null)
					r.Recycle();
				_free.Push(objList[i]);
			}
		}
	}
}
