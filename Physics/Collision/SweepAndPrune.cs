using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

using Henge3D.Collision;

namespace Henge3D.Collision
{
	/// <summary>
	/// Provides a sweep-and-prune broad phase collision detection implementation.
	/// </summary>
	/// <remarks>
	/// All objects are kept sorted along each axis and the results are collated to determine collision candidate pairs.
	/// </remarks>
	public class SweepAndPrune : BroadPhase
	{
		private const int DefaultCapacity = 128;
		private const int DefaultMatrixGrowFactor = 2;

		private TaskManager _tasks;
		private List<Composition> _list, _axisX, _axisY, _axisZ;
		private uint[,] _matrixX, _matrixY, _matrixZ;
		private int _newItems = 0, _rows, _columns, _growFactor = DefaultMatrixGrowFactor;

		public SweepAndPrune()
		{
			this.SetCapacity(DefaultCapacity);
			_list = new List<Composition>();
			_axisX = new List<Composition>();
			_axisY = new List<Composition>();
			_axisZ = new List<Composition>();
			_tasks = TaskManager.Current;
		}

		public int Capacity { get { return _rows; } set { this.SetCapacity(value); } }
		public int GrowFactor { get { return _growFactor; } set { _growFactor = value; } }

		public override void Add(Composition c)
		{
			c.Flags = (uint)_list.Count;
			_list.Add(c);
			_axisX.Add(c);
			_axisY.Add(c);
			_axisZ.Add(c);
			_newItems++;
		}

		public override void Remove(Composition c)
		{
			int pos = _list.IndexOf(c);
			if (pos < 0)
				return;
			if (_list.Count < 2)
				_list.RemoveAt(pos);
			else
			{
				_list[pos] = _list[_list.Count - 1];
				_list[pos].Flags = (uint)pos;
				_list.RemoveAt(_list.Count - 1);
			}
			_axisX.Remove(c);
			_axisY.Remove(c);
			_axisZ.Remove(c);
		}

		public override void Clear()
		{
			_list.Clear();
			_axisX.Clear();
			_axisY.Clear();
			_axisZ.Clear();
		}

		public override void Execute(CollisionFunctor cf)
		{
			while (_axisX.Count > _rows)
			{
				SetCapacity(_rows * _growFactor);
			}

			_tasks.AddTask(SortAndSweepX);
			_tasks.AddTask(SortAndSweepY);
			_tasks.AddTask(SortAndSweepZ);
			_tasks.Execute();

			for (int i = 0; i < _rows; i++)
			{
				for (int j = 0; j < _columns; j++)
				{
					uint block = _matrixX[i, j] & _matrixY[i, j] & _matrixZ[i, j];
					if (block > 0)
					{
						for (int k = 0; k < sizeof(uint); k++)
						{
							if (((1 << k) & block) > 0)
							{
								_tasks.AddTask(cf.WritePair, _list[i], _list[j * sizeof(uint) + k]);
							}
						}
					}
				}
			}
			_tasks.Execute();
		}

		#region SortAndSweep methods for X, Y, and Z

		private void SortAndSweepX(TaskParams parameters)
		{
			Array.Clear(_matrixX, 0, _matrixX.Length);

			if (_newItems > 0)
				_axisX.Sort(_xCompare);
			else
				InsertSortX();

			for (int i = 0; i < _axisX.Count; i++)
			{
				for (int j = i + 1; j < _axisX.Count; j++)
				{
					Composition ci = _axisX[i], cj = _axisX[j];
					if (ci.BoundingBox.Maximum.X >= cj.BoundingBox.Minimum.X)
					{
						int row = (int)(ci.Flags < cj.Flags ? ci.Flags : cj.Flags);
						int col = (int)(ci.Flags < cj.Flags ? cj.Flags : ci.Flags);
						_matrixX[row, col / sizeof(uint)] |= (uint)(1 << (col % sizeof(uint)));
					}
					else
						break;
				}
			}
		}

		private void SortAndSweepY(TaskParams parameters)
		{
			Array.Clear(_matrixY, 0, _matrixY.Length);

			if (_newItems > 0)
				_axisY.Sort(_yCompare);
			else
				InsertSortY();

			for (int i = 0; i < _axisY.Count; i++)
			{
				for (int j = i + 1; j < _axisY.Count; j++)
				{
					Composition ci = _axisY[i], cj = _axisY[j];
					if (ci.BoundingBox.Maximum.Y >= cj.BoundingBox.Minimum.Y)
					{
						int row = (int)(ci.Flags < cj.Flags ? ci.Flags : cj.Flags);
						int col = (int)(ci.Flags < cj.Flags ? cj.Flags : ci.Flags);
						_matrixY[row, col / sizeof(uint)] |= (uint)(1 << (col % sizeof(uint)));
					}
					else
						break;
				}
			}
		}

		private void SortAndSweepZ(TaskParams parameters)
		{
			Array.Clear(_matrixZ, 0, _matrixZ.Length);

			if (_newItems > 0)
				_axisZ.Sort(_zCompare);
			else
				InsertSortZ();

			for (int i = 0; i < _axisZ.Count; i++)
			{
				for (int j = i + 1; j < _axisZ.Count; j++)
				{
					Composition ci = _axisZ[i], cj = _axisZ[j];
					if (ci.BoundingBox.Maximum.Z >= cj.BoundingBox.Minimum.Z)
					{
						int row = (int)(ci.Flags < cj.Flags ? ci.Flags : cj.Flags);
						int col = (int)(ci.Flags < cj.Flags ? cj.Flags : ci.Flags);
						_matrixZ[row, col / sizeof(uint)] |= (uint)(1 << (col % sizeof(uint)));
					}
					else
						break;
				}
			}
		}

		#endregion

		#region sorting machinery

		private class AxisXComparer : Comparer<Composition>
		{
			public override int Compare(Composition x, Composition y)
			{
				float d = x.BoundingBox.Minimum.X - y.BoundingBox.Minimum.X;
				return d == 0f ? 0 : (d < 0f ? -1 : 1);
			}
		}

		private class AxisYComparer : Comparer<Composition>
		{
			public override int Compare(Composition x, Composition y)
			{
				float d = x.BoundingBox.Minimum.Y - y.BoundingBox.Minimum.Y;
				return d == 0f ? 0 : (d < 0f ? -1 : 1);
			}
		}

		private class AxisZComparer : Comparer<Composition>
		{
			public override int Compare(Composition x, Composition y)
			{
				float d = x.BoundingBox.Minimum.Z - y.BoundingBox.Minimum.Z;
				return d == 0f ? 0 : (d < 0f ? -1 : 1);
			}
		}

		private static AxisXComparer _xCompare = new AxisXComparer();
		private static AxisYComparer _yCompare = new AxisYComparer();
		private static AxisZComparer _zCompare = new AxisZComparer();

		private void InsertSortX()
		{
			for (int i = 0; i < _axisX.Count; i++)
			{
				int j = i - 1;
				while (j >= 0 && _axisX[j].BoundingBox.Minimum.X > _axisX[i].BoundingBox.Minimum.X)
				{
					_axisX[j + 1] = _axisX[j];
					--j;
				}
				_axisX[j + 1] = _axisX[i];
			}
		}

		private void InsertSortY()
		{
			for (int i = 0; i < _axisY.Count; i++)
			{
				int j = i - 1;
				while (j >= 0 && _axisY[j].BoundingBox.Minimum.Y > _axisY[i].BoundingBox.Minimum.Y)
				{
					_axisY[j + 1] = _axisY[j];
					--j;
				}
				_axisY[j + 1] = _axisY[i];
			}
		}

		private void InsertSortZ()
		{
			for (int i = 0; i < _axisZ.Count; i++)
			{
				int j = i - 1;
				while (j >= 0 && _axisZ[j].BoundingBox.Minimum.Z > _axisZ[i].BoundingBox.Minimum.Z)
				{
					_axisZ[j + 1] = _axisZ[j];
					--j;
				}
				_axisZ[j + 1] = _axisZ[i];
			}
		}

		#endregion

		private void SetCapacity(int size)
		{
			_rows = size;
			_columns = _rows / sizeof(uint) + 1;
			_matrixX = new uint[_rows, _columns];
			_matrixY = new uint[_rows, _columns];
			_matrixZ = new uint[_rows, _columns];
		}

		public override Composition Intersect(ref Segment segment, out float scalar, out Vector3 point)
		{
			var axis = _axisX;
			scalar = float.PositiveInfinity;
			point = Vector3.Zero;
			float scalar1;
			Vector3 point1;
			Composition ret = null;
			for (int i = 0; i < axis.Count; i++)
			{
				if (axis[i].Intersect(ref segment, out scalar1, out point1) && scalar1 < scalar)
				{
					scalar = scalar1;
					point = point1;
					ret = axis[i];
				}
			}
			return ret;
		}
	}
}
