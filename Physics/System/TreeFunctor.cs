using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Henge3D
{
	internal abstract class TreeFunctor<Tnode,Tindex>
	{
		public Tindex[] Stack;
		public Tnode[] Buffer;

		int _stackSize;

		public TreeFunctor(int bufferSize, int stackSize)
		{
			if (bufferSize < 1) throw new ArgumentException("Buffer size cannot be less than one.", "bufferSize");
			if (stackSize < 1) throw new ArgumentException("Stack size cannot be less than one.", "stackSize");

			Stack = new Tindex[stackSize];
			Buffer = new Tnode[bufferSize];
			_stackSize = stackSize;
		}

		public int GrowStack()
		{
			int curStackSize = Stack.Length;
			var newStack = new Tindex[curStackSize + _stackSize];
			Array.Copy(Stack, newStack, curStackSize);
			Stack = newStack;
			return Stack.Length;
		}

		public abstract void Process(int count);
	}
}
