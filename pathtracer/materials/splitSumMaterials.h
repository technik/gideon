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

#include "MicrosurfaceScattering.h"

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
	) const override = 0;

protected:

	math::Vec3f iblLookUp(const math::Ray& in, HitRecord& hit, float roughness) const
	{
		auto ndv = dot(-in.direction(), hit.normal);
		return m_iblSampler.sample({ ndv, roughness });
	}

	EnvironmentProbe* m_env;
	BilinearTextureSampler<ClampWrap,ClampWrap> m_iblSampler;
};

class SplitSumReflectorSS : public SplitSumMaterial
{
public:
	SplitSumReflectorSS(EnvironmentProbe* probe, float roughness)
		: SplitSumMaterial(probe)
		, r(roughness)
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
		auto Ess = f_ab.x() + f_ab.y();
		auto reflDir = reflect(in.direction(), hit.normal);
		emitted = Ess * m_env->radiance(reflDir, r);
		return false; // Do not scatter the ray further
	}

private:
	float r;
};

class SplitSumReflectorMS : public SplitSumMaterial
{
public:
	SplitSumReflectorMS(EnvironmentProbe* probe, float roughness)
		: SplitSumMaterial(probe)
		, r(roughness)
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
		auto Ess = f_ab.x() + f_ab.y();
		auto Ems = 1.f - Ess;

		auto reflDir = reflect(in.direction(), hit.normal);
		emitted = Ess * m_env->radiance(reflDir, r) + Ems * m_env->irradiance(hit.normal);
		return false; // Do not scatter the ray further
	}

private:
	float r;
};

class CopperSS : public SplitSumMaterial
{
public:
	CopperSS(EnvironmentProbe* probe, float roughness)
		: SplitSumMaterial(probe)
		, r(roughness)
		, F0(0.95f, 0.64f, 0.54f)
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
		emitted = (F0*f_ab.x() + math::Vec3f(f_ab.y())) * m_env->radiance(reflDir, r);
		return false; // Do not scatter the ray further
	}

private:
	math::Vec3f F0;
	float r;
};

class HeitzReflector : public SplitSumMaterial
{
public:
	HeitzReflector(EnvironmentProbe* probe, float roughness)
		: SplitSumMaterial(probe)
		, r(roughness)
		, ax(roughness*roughness)
		, ay(roughness*roughness)
		, m_reflector(false, false, ax, ay)
	{
	}

	bool scatter(
		const math::Ray& in,
		HitRecord& hit,
		math::Vec3f&,// attenuation,
		math::Vec3f& emitted,
		math::Ray&,// out,
		RandomGenerator& random
	) const override
	{
		// Tangent space
		math::Vec3f tan, bit;
		branchlessONB(hit.normal, tan, bit);
		math::Vec3f wsEye = normalize(-in.direction());

		// Transform view dir to local space
		math::Vec3f tsEye(dot(wsEye, tan), dot(wsEye, bit), dot(wsEye, hit.normal));

		// Random out vector in the hemisphere
		math::Vec3f tsReflDir = random.unit_vector();
		tsReflDir.z() = abs(tsReflDir.z());
		/*math::Vec3f tsIn = random.unit_vector();
		tsIn.z() = abs(tsIn.z());
		auto intensity = m_reflector.evalSingleScattering(tsIn, tsReflDir);*/
		//auto intensity = m_reflector.evalPhaseFunction(tsEye, tsReflDir);

		double value_quadrature = 0;
		constexpr double thetaStep = 0.05;
		constexpr double phiStep = 0.05;
		for (double theta_o = 0; theta_o < math::Pi; theta_o += thetaStep)
			for (double phi_o = 0; phi_o < math::TwoPi; phi_o += phiStep)
			{
				const math::Vec3f wo(cos(phi_o)*sin(theta_o), sin(phi_o)*sin(theta_o), cos(theta_o));
				value_quadrature += thetaStep * phiStep * abs(sin(theta_o)) * (double)m_reflector.evalSingleScattering(tsEye, wo);
				//value_quadrature += thetaStep*phiStep*abs(sin(theta_o)) * (double)m_reflector.evalPhaseFunction(tsEye, wo);
			}

		// Transform reflection direction back to world space
		math::Vec3f wsRefl = tsReflDir.x() * tan + tsReflDir.y() * bit + tsReflDir.z() * hit.normal;
		emitted = value_quadrature * m_env->radiance(wsRefl, r);
		//emitted = intensity * m_env->radiance(wsRefl, r);

		return false; // Do not scatter the ray further
	}

	// Pixar's method for orthonormal basis generation
	static void branchlessONB(const math::Vec3f &n, math::Vec3f &b1, math::Vec3f& b2)
	{
		float sign = copysignf(1.0f, n.z());
		const float a = -1.0f / (sign + n.z());
		const float b = n.x() * n.y() * a;
		b1 = math::Vec3f(1.0f + sign * n.x() * n.x() * a, sign * b, -sign * n.x());
		b2 = math::Vec3f(b, sign + n.y() * n.y() * a, -n.y());
	}

private:
	math::Vec3f F0;
	float r;
	float ax, ay;
	MicrosurfaceConductor m_reflector;
};
