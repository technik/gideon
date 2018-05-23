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
		, uWrapper(img->width())
		, vWrapper(img->height())
	{}

	BilinearTextureSampler(const char* imgFileName)
		: BilinearTextureSampler(std::make_shared<Image>(imgFileName))
	{}

	/// Texture coordinates start at the upper left corner
	math::Vec3f sample(float u, float v) const
	{
		// transform u-v coordinates
		u = u*mImg->width();
		auto u0 = floor(u);
		auto u1 = u0+1;
		v = (1.f-v)*mImg->height(); // Invert v to properly read in image coordinates
		auto v0 = floor(v);
		auto v1 = v0+1;
		// Get the relevant pixels
		auto& a = mImg->pixel(uWrapper(u0),vWrapper(v0));
		auto& b = mImg->pixel(uWrapper(u1),vWrapper(v0));
		auto& c = mImg->pixel(uWrapper(u0),vWrapper(v1));
		auto& d = mImg->pixel(uWrapper(u1),vWrapper(v1));
		auto du = u-u0;
		auto dv = v-v0;
		auto top = a*(1-du)+b*du;
		auto bottom = c*(1-du)+d*du;
		return top*(1-dv)+bottom*dv;
	}

private:
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
		auto mod = int(std::floorf(x*mSize));
		return mod%mSize;
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
		auto raw = int(std::floorf(x*mSize));
		return std::min(
			(size_t)std::max(raw, 0),
			mSize-1);
	}

private:
	size_t mSize;
};