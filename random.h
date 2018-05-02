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

#include <random>
#include <math/vector3.h>

class RandomGenerator
{
public:
	float scalar()
	{
		return distrib(engine);
	}
	
	math::Vec3f unit_vector()
	{
		math::Vec3f p;
		do
		{
			p = 2*math::Vec3f(scalar(),scalar(),scalar())-1.f;
		} while(p.sqNorm() >= 1.f);
		return p;
	}
private:
	std::default_random_engine engine;
	std::uniform_real_distribution<float> distrib;
};
