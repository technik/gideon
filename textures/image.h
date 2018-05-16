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

#include <cassert>
#include <cmath>
#include <functional>
#include <math/vector3.h>
#include <memory>
#include <stb_image.h>

//--------------------------------------------------------------------------------------------------
class Image
{
	using img_deleter = std::function<void(void*)>;
	using data_ptr = std::unique_ptr<math::Vec3f[], img_deleter>;
public:
	Image(const char* fileName)
	{
		int nComponents;
		int isx, isy;
		auto rawData = stbi_loadf(fileName, &isx, &isy, &nComponents, 3);
		assert(isx >= 0 && isy >= 0);
		sx = size_t(isx);
		sy = size_t(isy);
		mData = data_ptr(reinterpret_cast<math::Vec3f*>(rawData), stbi_image_free);
	}

	// Coordinates starting in the upper left corner
	const math::Vec3f& pixel(size_t x, size_t y) const
	{
		return mData[x+sx*y];
	}

	math::Vec3f& pixel(int x, int y)
	{
		return const_cast<math::Vec3f&>(((const Image*)this)->pixel(x,y));
	}

private:
	void sampleSpherical(const math::Vec3f& dir, int& x, int& y) const
	{
		float u = atan2(dir.z(), -dir.x()) * 0.1591f + 0.5f;
		float v = asin(-dir.y()) * 0.3183f + 0.5f;
		x = int(std::clamp(u, 0.f, 1.f)*sx);
		y = int(std::clamp(v, 0.f, 1.f)*sy);
	}

	size_t sx, sy;
	data_ptr mData;
};