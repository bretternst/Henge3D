using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Henge3D
{
	public interface IAllocator<T> where T : class, new()
	{
		T Allocate();
		void Recycle(T obj);
		void Recycle(IList<T> objList);
	}
}
