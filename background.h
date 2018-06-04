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

#include <math/vector2.h>
#include <math/vector3.h>
#include <textures/image.h>
#include <textures/textureSampler.h>
#include <cmath>

//--------------------------------------------------------------------------------------------------
class Background
{
public:
	virtual math::Vec3f sample(const math::Vec3f& dir) const = 0;
};

//--------------------------------------------------------------------------------------------------
class GradientBackground : public Background
{
public:
	GradientBackground(const math::Vec3f& upColor, const math::Vec3f& downColor)
		: mUpColor(upColor)
		, mDownColor(downColor)
	{}

	math::Vec3f sample(const math::Vec3f& direction) const override
	{
		float f = 0.5f + 0.5f * direction.y();
		return mUpColor*f + (1-f)*mDownColor;
	}

private:
	math::Vec3f mUpColor, mDownColor;
};

//--------------------------------------------------------------------------------------------------
class HDRBackground : public Background
{
public:
	HDRBackground(const char* fileName)
		: mSampler(fileName)
	{
	}

	math::Vec3f sample(const math::Vec3f& direction) const override
	{
		// Transform direction into uv coordinates
		auto uv = sampleSpherical(direction);
		return mSampler.sample(uv);
	}

private:
	using Sampler = BilinearTextureSampler<RepeatWrap,ClampWrap>;

	math::Vec2f sampleSpherical(const math::Vec3f& dir) const
	{
		return {
			atan2(dir.z(), -dir.x()) * 0.1591f + 0.5f,
			asin(dir.y()) * 0.3183f + 0.5f
		};
	}

	Sampler mSampler;
};