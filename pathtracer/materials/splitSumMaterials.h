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
// Materials using the split sum approximation and prefiltered environment probes

#include "material.h"
#include <textures/environmentProbe.h>
#include <textures/textureSampler.h>
#include <textures/image.h>

class SplitSumMaterial : public Material
{
public:
	SplitSumMaterial(EnvironmentProbe* probe)
		: m_env(probe)
		, m_iblSampler("ibl.hdr")
	{
	}

	bool scatter(
		const math::Ray& in,
		HitRecord& hit,
		math::Vec3f&,// attenuation,
		math::Vec3f& emitted,
		math::Ray&,// out,
		RandomGenerator&// random
	) const override
	{
		auto f_ab = iblLookUp(in, hit, r);
		auto reflDir = reflect(in.direction(), hit.normal);
		emitted = math::Vec3f(f_ab.x() + f_ab.y()) * m_env->radiance(reflDir, r);
		return false; // Do not scatter the ray
	}

protected:

	math::Vec3f iblLookUp(const math::Ray& in, HitRecord& hit, float roughness) const
	{
		auto ndv = dot(-in.direction(), hit.normal);
		return m_iblSampler.sample({ ndv, r });
	}

	float r = 0.5f;
	EnvironmentProbe* m_env;
	BilinearTextureSampler<ClampWrap,ClampWrap> m_iblSampler;
};
