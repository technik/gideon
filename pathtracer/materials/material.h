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

#include <math/ray.h>
#include <math/vector.h>
#include "collision.h"
#include "math/random.h"
#include "textures/textureSampler.h"

class Material
{
public:
	virtual bool scatter(const math::Ray& in, HitRecord& record, math::Vec3f& attenuation, math::Ray& out, RandomGenerator& random) const = 0;
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
