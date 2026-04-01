#pragma once
#include "Defines.h"
#include "MazeMapEigen.h"
#include "Vector2f.h"
#include <cmath>
namespace MazeMap
{
	// Use Eigen implementation.
	template<int ROWS, int COLS>
	using Matrixf = Eigen::Matrix<float,ROWS,COLS>;
	template<int DIMENSION>
	using Vectorf = Eigen::Vector<float, DIMENSION>;
	/*
	template<int ROWS, int COLS>
	class Matrixf
	{
	private:
		Vectorf<COLS> _data[ROWS];

	public:
		Matrixf()
			: _data()
		{
			for (uint8_t i = 0; i < ROWS && i < COLS; i++)
			{
				_data[i][i] = 1.0f;
			}
		}

		Matrixf(const float (&data)[ROWS][COLS])
			: _data()
		{
			for (uint8_t i = 0; i < ROWS; i++)
			{
				for (uint8_t j = 0; j < COLS; j++)
				{
					_data[i][j] = data[i][j];
				}
			}
		}

		float& operator()(uint8_t row, uint8_t col) { return _data[row][col]; }
		const float& operator()(uint8_t row, uint8_t col) const { return _data[row][col]; }

		template<int OTHER_COL>
		void Times(const Matrixf<COLS, OTHER_COL>& other, Matrixf<ROWS, OTHER_COL>& result) const
		{
			for (uint8_t i = 0; i < OTHER_COL; i++)
			{
				for (uint8_t j = 0; j < ROWS; j++)
				{
					result(i, j) = 0.0f;
					for (uint8_t k = 0; k < COLS; k++)
					{
						result(i, j) += (*this)(j,k) * other(k,i);
					}
				}
			}
		}
		template<int OTHER_COL>
		void Times(const Matrixf<COLS, OTHER_COL>& other, Matrixf<ROWS, OTHER_COL>& result)
		{
			return const_cast<const Matrixf<ROWS, COLS>*>(this)->Times(other, result);
		}

		void Times(const Vectorf<COLS>& vec, Vectorf<ROWS>& result)
		{
			for (uint8_t j = 0; j < ROWS; j++)
			{
				result[j] = 0.0f;
				for (uint8_t k = 0; k < COLS; k++)
				{
					result[j] += (*this)(j, k) * vec[k];
				}
			}
		}

		void Plus(const Matrixf<ROWS, COLS>& other, Matrixf<ROWS,COLS> result)
		{
			for (uint8_t i = 0; i < ROWS; i++)
			{
				for (uint8_t j = 0; j < COLS; j++)
				{
					result(i, j) = (*this)(i, j) + other(i, j);
				}
			}
		}

		void Minus(const Matrixf<ROWS, COLS>& other, Matrixf<ROWS, COLS> result)
		{
			for (uint8_t i = 0; i < ROWS; i++)
			{
				for (uint8_t j = 0; j < COLS; j++)
				{
					result(i, j) = (*this)(i, j) - other(i, j);
				}
			}
		}

		void PlusEqual(const Matrixf<ROWS, COLS>& other)
		{
			for (uint8_t i = 0; i < ROWS; i++)
			{
				for (uint8_t j = 0; j < COLS; j++)
				{
					(*this)(i, j) += other(i, j);
				}
			}
		}
	};

	template<int ROWS>
	class Matrixsf : public Matrixf<ROWS, ROWS>
	{
	public:
		Matrixsf()
			: Matrixf<ROWS, ROWS>()
		{
		}
		Matrixsf(const float(&data)[ROWS][ROWS])
			: Matrixf<ROWS, ROWS>(data)
		{}

		void CholeskyDecomp(Matrixf<ROWS, ROWS>& result)
		{
			for (uint8_t i = 0; i < ROWS; i++)
			{
				for (uint8_t j = 0; j <= i; j++)
				{
					float sum = 0.0f;
					for (uint8_t k = 0; k < j; k++)
					{
						sum += result(i, k)* result(j, k);
					}
					if (i > j)
					{
						result(i, j) = ((*this)(i, j) - sum) / result(j, j);
					}
					else
					{
						result(i, j) = MazeMap::Math::Absf(MazeMap::Math::Sqrtf((*this)(i, i) - sum));
					}
				}
				for (uint8_t j = i+1; j < ROWS; j++)
				{
					result(i, j) = 0.0f;
				}
			}
		}
	};*/
}



