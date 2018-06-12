//-------------------------------------------------------------------------------------------------
// Toy path tracer
//--------------------------------------------------------------------------------------------------
// Copyright 2018 Carmelo J Fdez-Aguera
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software
// and associated documentation files (the "Software"), to deal in the Software without restriction,
// including without limitation the rights to use, copy, modify, merge, publish, distribute,
// sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all copies or
// substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
// NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
// NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#pragma once

#include <array>
#include <cassert>
#include "vector.h"

namespace math
{
	class Matrix34f
	{
	public:
		Matrix34f() = default;
		Matrix34f(const std::array<float,16>& x)
			: m(x)
		{}

		Matrix34f(float x)
		{
			for(auto i = 0; i < 16; ++i)
				m[i] = x;
		}

		Matrix34f inverse() const;

		static Matrix34f identity()
		{
			Matrix34f x;
			for(int i = 0; i < 4; ++i)
				for(int j = 0; j < 4; ++j)
					x(i,j) = float(i==j?1:0);
			return x;
		}

		Vec3f& position() { return reinterpret_cast<Vec3f&>((*this)(0,3)); }
		Vec3f position() const {
			return reinterpret_cast<const Vec3f&>(m[4*3]);
		}

		Matrix34f operator*(const Matrix34f& b) const
		{
			Matrix34f res;
			for(int i = 0; i < 3; ++i)
			{
				for(int j = 0; j < 4; ++j)
				{
					res(i,j) =
						(*this)(i,0)*b(0,j) +
						(*this)(i,1)*b(1,j) +
						(*this)(i,2)*b(2,j) +
						(*this)(i,3);
				}
			}
			res(3,0) = 0.f;
			res(3,1) = 0.f;
			res(3,2) = 0.f;
			res(3,3) = 1.f;
			return res;
		}

		Vec3f transformPos(const Vec3f& v) const
		{
			Vec3f res;
			for(int i = 0; i < 3; ++i)
			{
				res[i] =
					(*this)(i,0)*v[0] +
					(*this)(i,1)*v[1] +
					(*this)(i,2)*v[2] +
					(*this)(i,3);
			}
			return res;
		}

		Vec3f transformDir(const Vec3f& v) const
		{
			Vec3f res;
			for(int i = 0; i < 3; ++i)
			{
				res[i] =
					(*this)(i,0)*v[0] +
					(*this)(i,1)*v[1] +
					(*this)(i,2)*v[2];
			}
			return res;
		}

		float& operator()(int i, int j)
		{
			return m[4*j+i];
		}

		float operator()(int i, int j) const
		{
			return m[4*j+i];
		}

	private:
		std::array<float,16> m;
	};

	class Matrix44f
	{
	public:
		Matrix44f() = default;
		Matrix44f(const std::array<float,16>& x)
			: m(x)
		{}

		static std::array<float,4> lowSolve(const Matrix44f& L, std::array<float,4>& y)
		{
			std::array<float,4> x;
			x[0] = y[0];
			x[1] = y[1] - x[0]*L(1,0);
			x[2] = y[2] - x[0]*L(2,0) - x[1]*L(2,1);
			x[3] = y[3] - x[0]*L(3,0) - x[1]*L(3,1) - x[2]*L(3,2);
			return x;
		}

		static std::array<float,4> upSolve(const Matrix44f& L, std::array<float,4>& y)
		{
			std::array<float,4> x;
			x[3] =  y[3]/L(3,3);
			x[2] = (y[2] - x[3]*L(2,3)) /L(2,2);
			x[1] = (y[1] - x[3]*L(1,3) - x[2]*L(1,2))/L(1,1);
			x[0] = (y[0] - x[3]*L(0,3) - x[2]*L(0,2) - x[1]*L(0,1))/L(0,0);
			return x;
		}

		Matrix44f inverse() const
		{
			Matrix44f inv;
			Matrix44f L, U;
			std::array<int,4> P;
			factorizationLU(L,U,P);
			for(int j = 0; j < 4; ++j)
			{
				std::array<float,4> b = {};
				b[P[j]] = 1.f;
				auto y = lowSolve(L,b);
				auto x = upSolve(U,y);
				for(auto i = 0; i < 4; ++i)
					inv(i,j) = x[i];
			}
			return inv;
		}

		Matrix44f(const Matrix34f& x)
		{
			*this = Matrix44f::identity();
			for(int i = 0; i < 3; ++i)
				for(int j = 0; j < 4; ++j)
					(*this)(i,j) = x(i,j);
		}

		bool operator== (const Matrix44f& x) const
		{
			for(int i = 0; i < 3; ++i)
				for(int j = 0; j < 4; ++j)
					if((*this)(i,j) != x(i,j)) return false;
			return true;
		}

		static Matrix44f identity()
		{
			Matrix44f x;
			for(int i = 0; i < 4; ++i)
				for(int j = 0; j < 4; ++j)
					x(i,j) = float(i==j?1:0);
			return x;
		}

		Matrix44f operator*(const Matrix44f& b) const
		{
			Matrix44f res;
			for(int i = 0; i < 4; ++i)
			{
				for(int j = 0; j < 4; ++j)
				{
					res(i,j) =
						(*this)(i,0)*b(0,j) +
						(*this)(i,1)*b(1,j) +
						(*this)(i,2)*b(2,j) +
						(*this)(i,3)*b(3,j);
				}
			}
			return res;
		}

		float& operator()(int i, int j)
		{
			return m[4*j+i];
		}

		float operator()(int i, int j) const
		{
			return m[4*j+i];
		}

		float element(int i, int j) const
		{
			return m[4*j+i];
		}

		// Apply gauss elimination with partial pivoting
		void factorizationLU(Matrix44f& L, Matrix44f& U, std::array<int,4>& P) const
		{
			P = {0, 1, 2, 3};
			L = identity();
			U = *this;
			// For each column
			for(auto k = 0; k < 4; k++)
			{
				// Find best pivot
				auto maxA = abs(element(k,k));
				auto bestI = P[k];
				for(auto i = k+1; i < 4; ++i) // Find best pivot in the column
				{
					auto absA = abs(element(i,k));
					if(absA > maxA)
					{
						maxA = absA;
						bestI = P[i];
					}
				}
				constexpr float tolerance = 1e-4f;
				assert(maxA > tolerance); // Ensure minimun conditioning of the matrices
				P[k] = bestI;

				// permutate pivot row
				if(k != P[k])
					for(auto j = k+1; j < 4; ++j) // Swap all elements to the right of the pivot, in both rows k and p[k]
					{
						auto a = U(j,k);
						U(j,k) = U(j,P[k]);
						U(j,P[k]) = a;
					}

				// Partially solve all rows below the pivot
				for(auto i = k+1; i < 4; ++i)
				{
					auto pivot = U(k,k);
					auto rowCoeff = U(i,k) / pivot; // Multiplier coefficient for row i
					L(i,k) = rowCoeff;
					for(auto j = i+1; j < 4; ++j)
					{
						U(i,j) = U(i,j) - rowCoeff*U(k,j);
					}
					U(i,k) = 0; // U is upper triangular, so all elements below the diagonal must be 0
				}
			}
		}

	private:
		std::array<float,16> m;
	};

	inline Matrix34f Matrix34f::inverse() const
	{
		Matrix34f inv;
		auto x = Matrix44f(*this);
		auto xi = x.inverse();
		for(int j = 0; j < 4; ++j)
			for(int i = 0; i < 4; ++i)
				inv(i,j) = xi(i,j);
		return inv;
	}
}