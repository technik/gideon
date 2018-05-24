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

#include <math/ray.h>
#include <math/vector3.h>
#include "collision.h"
#include "random.h"
#include "textures/textureSampler.h"

class Material
{
public:
	virtual bool scatter(const math::Ray& in, HitRecord& record, math::Vec3f& attenuation, math::Ray& out, RandomGenerator& random) const = 0;
};

class Lambertian : public Material
{
public:
	Lambertian(const math::Vec3f& c) : albedo(c) {}
	bool scatter(const math::Ray&, HitRecord& hit, math::Vec3f& attenuation, math::Ray& out, RandomGenerator& random) const override
	{
		auto target = hit.p + hit.normal + random.unit_vector();
		out = math::Ray(hit.p, target-hit.p);
		attenuation = albedo;
		return true;
	}

	math::Vec3f albedo;
};

class PBRMaterial : public Material
{
public:
	using Sampler = BilinearTextureSampler<RepeatWrap,RepeatWrap>;
	PBRMaterial(const math::Vec3f& baseColor, const std::shared_ptr<Sampler>& baseClrMap) : albedo(baseColor), albedoMap(baseClrMap) {}
	bool scatter(const math::Ray&, HitRecord& hit, math::Vec3f& attenuation, math::Ray& out, RandomGenerator& random) const override
	{
		auto target = hit.p + hit.normal + random.unit_vector();
		out = math::Ray(hit.p, target-hit.p);
		/*if(albedoMap)
			attenuation = albedoMap->sample(hit.u, hit.v);
		else
			attenuation = albedo;*/
		attenuation.r() = hit.u;
		attenuation.g() = hit.v;
		attenuation.b() = 0.f;
		return true;
	}

	math::Vec3f albedo;
	std::shared_ptr<Sampler> albedoMap;
};

class Metal : public Material
{
public:
	Metal(const math::Vec3f& c, float f) : albedo(c), fuzz(f) {}
	bool scatter(const math::Ray& in, HitRecord& hit, math::Vec3f& attenuation, math::Ray& out, RandomGenerator& random) const override
	{
		auto reflected = reflect(normalize(in.direction()), hit.normal);
		out = math::Ray(hit.p, reflected + random.unit_vector()*fuzz);
		attenuation = albedo;
		return dot(out.direction(), hit.normal) > 0.f;
	}

	math::Vec3f albedo;
	float fuzz;
};
