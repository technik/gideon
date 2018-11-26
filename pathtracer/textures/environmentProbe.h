//-------------------------------------------------------------------------------------------------
// Toy path tracer
//-------------------------------------------------------------------------------------------------
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
// Environment probe with prefiltered radiance and irradiance

#include <math/vector.h>

class EnvironmentProbe
{
public:
	virtual math::Vec3f irradiance(const math::Vec3f& direction) const = 0;
	virtual math::Vec3f radiance(const math::Vec3f& direction, float lod) const = 0;
};

class SplitSumProbe : public EnvironmentProbe
{
public:
	SplitSumProbe(const char* fileName);
	math::Vec3f irradiance(const math::Vec3f& direction) const override;
	math::Vec3f radiance(const math::Vec3f& direction, float lod) const override;
};

class FurnaceProbe : public EnvironmentProbe
{
public:
	FurnaceProbe(float intensity) : m_intensity(intensity) {}

	math::Vec3f irradiance(const math::Vec3f& direction) const override { return m_intensity; }
	math::Vec3f radiance(const math::Vec3f& direction, float lod) const override { return m_intensity; }

private:
	math::Vec3f m_intensity;
};