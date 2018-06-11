//-------------------------------------------------------------------------------------------------
// Toy path tracer
//-------------------------------------------------------------------------------------------------
// Based on the minibook 'Raytracing in one weekend' and Aras P.'s series: Daily pathtracer
// https://aras-p.info/blog/
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

		Matrix34f inverse() const
		{
			Matrix34f inv;
			// TODO: Handle scales
			for(int i = 0; i < 3; ++i)
			{
				for(int j = 0; j < 3; ++j)
				{
					inv(i,j) = (*this)(j,i);
				}
			}
			inv(3,0) = 0.f;
			inv(3,1) = 0.f;
			inv(3,2) = 0.f;
			inv(3,3) = 1.f;
			auto invPos = -(inv.transformDir(position()));
			inv.position() = invPos;
			return inv;
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
}