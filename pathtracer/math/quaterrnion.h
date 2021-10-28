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
#include "matrix.h"
#include "vector.h"
#include <cmath>

namespace math
{
	class Quatf
	{
	public:
		Quatf() = default;
		constexpr Quatf(std::array<float,4> _list) : m(_list) {}
		Quatf(std::initializer_list<float> il)
		{
			auto iter = il.begin();
			for(size_t i = 0; i < 4; ++i)
				m[i] = *iter++;
		}

		Matrix34f rotationMtx() const
		{
			auto& x = m[0];
			auto& y = m[1];
			auto& z = m[2];
			auto& w = m[3];
			auto a2 = w*w;
			auto b2 = x*x;
			auto c2 = y*y;
			auto d2 = z*z;
			auto ab = w*x;
			auto ac = w*y;
			auto ad = w*z;
			auto bc = x*y;
			auto bd = x*z;
			auto cd = y*z;
			return Matrix34f({
				a2+b2-c2-d2,2*(bc+ad), 2*(bd-ac), // Column 0
				2*(bc-ad),a2-b2+c2-d2, 2*(cd+ab), // Column 1
				2*(bd+ac), 2*(cd-ab), a2-b2-c2+d2, // Column 2
				0.f, 0.f, 0.f
				});
		}

	private:
		std::array<float,4> m;
	};
}