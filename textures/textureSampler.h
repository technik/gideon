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

#include <algorithm>
#include <cmath>
#include "image.h"
#include <math/linear.h>
#include <math/vector2.h>
#include <math/vector3.h>
#include <memory>

//--------------------------------------------------------------------------------------------------
template<class UPolicy, class VPolicy>
class BilinearTextureSampler
{
public:
	using img_ptr = std::shared_ptr<Image>;

	BilinearTextureSampler(img_ptr img)
		: mImg(img)
		, nx(img->width())
		, ny(img->height())
		, uWrapper(img->width())
		, vWrapper(img->height())
	{
		pixDu = 1.f/nx;
		pixDv = 1.f/ny;
	}

	BilinearTextureSampler(const char* imgFileName)
		: BilinearTextureSampler(std::make_shared<Image>(imgFileName))
	{}

	/// Texture coordinates start at the upper left corner
	math::Vec3f sample(const math::Vec2f& uv) const
	{
		// transform u-v coordinates into image space (pixel units)
		auto s = uv.x()*nx;
		auto t = uv.y()*ny;
		auto s0 = floor(s);
		auto t0 = floor(t);
		auto s1 = s0+1;
		auto t1 = t0+1;
		// Get the relevant pixels
		auto& a = mImg->pixel(uWrapper(s0),vWrapper(t0));
		auto& b = mImg->pixel(uWrapper(s1),vWrapper(t0));
		auto& c = mImg->pixel(uWrapper(s0),vWrapper(t1));
		auto& d = mImg->pixel(uWrapper(s1),vWrapper(t1));
		// Interpolate
		auto ds = s-s0;
		auto dt = t-t0;
		auto top = math::lerp(a, b, ds);
		auto bottom = math::lerp(c, d, ds);
		return math::lerp(top, bottom, dt);
	}

private:
	size_t nx, ny;
	float pixDu, pixDv;
	img_ptr mImg;
	UPolicy uWrapper;
	VPolicy vWrapper;
};

//--------------------------------------------------------------------------------------------------
struct RepeatWrap
{
	RepeatWrap(size_t x) : mSize(x) {}

	size_t operator()(float x) const
	{
		auto raw = int(std::floorf(x));
		return raw%mSize;
	}

private:
	size_t mSize;
};

//--------------------------------------------------------------------------------------------------
struct ClampWrap
{
	ClampWrap(size_t x) : mSize(x) {}

	size_t operator()(float x) const
	{
		auto raw = int(x);
		return std::min(
			(size_t)std::max(raw, 0),
			mSize-1);
	}

private:
	size_t mSize;
};