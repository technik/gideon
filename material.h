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
		auto target = hit.normal + random.unit_vector();
		out = math::Ray(hit.p, target);
		attenuation = albedo;
		return true;
	}

	math::Vec3f albedo;
};

class PBRMaterial : public Material
{
public:
	using Sampler = BilinearTextureSampler<RepeatWrap,RepeatWrap>;
	PBRMaterial(
		const math::Vec3f& baseColor,
		const std::shared_ptr<Sampler>& baseClrMap,
		const std::shared_ptr<Sampler>& _physicsMap,
		const std::shared_ptr<Sampler>& _aoMap)
		: albedo(baseColor), albedoMap(baseClrMap), physicsMap(_physicsMap), aoMap(_aoMap)
	{}

	static math::Vec3f fresnelSchlick(float cosTheta, const math::Vec3f& F0)
	{
		return F0 + (1.f - F0) * pow(1.f - cosTheta, 5.f);
	}

	bool scatter(const math::Ray& in, HitRecord& hit, math::Vec3f& attenuation, math::Ray& out, RandomGenerator& random) const override
	{
		//auto ao = aoMap->sample(hit.u, hit.v).r(); // b is metalness
		auto roughness = physicsMap ? physicsMap->sample(hit.u, hit.v).g() : 1.f; // b is metalness
		auto metalness = physicsMap ? physicsMap->sample(hit.u, hit.v).b() : 1.f; // b is metalness
					
		math::Vec3f baseColor;
		if(albedoMap)
			baseColor = albedoMap->sample(hit.u, hit.v);
		else
			baseColor = albedo;

		math::Vec3f specColor = lerp(math::Vec3f(0.04f), baseColor, metalness);
		math::Vec3f diffColor = baseColor*(1.0f-metalness);

		auto ndh = std::max(0.f,-dot(hit.normal, in.direction()));
		auto Fresnel = fresnelSchlick(ndh, specColor);
		auto kS = Fresnel.norm();

		math::Vec3f scatterDir;
		auto diffDir = hit.normal + random.unit_vector();
		if(random.scalar() > kS) // Diffuse
		{
			attenuation = diffColor*ndh;
			scatterDir = diffDir*(math::Vec3f(1.f)-Fresnel);
		} else {// Specular
			auto m = normalize(hit.normal + random.unit_vector()*roughness);
			scatterDir = reflect(in.direction(), m);
			auto mdh = std::max(0.f,-dot(m, in.direction()));
			attenuation = specColor * mdh;
		}

		out = math::Ray(hit.p, scatterDir);
		attenuation = lerp(baseColor, {1.f,1.f,1.f}, metalness);

		return true;
	}

	math::Vec3f albedo;
	std::shared_ptr<Sampler> albedoMap;
	std::shared_ptr<Sampler> physicsMap;
	std::shared_ptr<Sampler> aoMap;
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
