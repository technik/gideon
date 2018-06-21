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

#include "material.h"

class Lambertian : public Material
{
public:
	Lambertian(const math::Vec3f& c) : albedo(c) {}
	bool scatter(const math::Ray& ray, HitRecord& hit, math::Vec3f& attenuation, math::Ray& out, RandomGenerator& random) const override
	{
		/*attenuation = math::Vec3f(hit.t/5.f);//hit.normal;
		return true;*/
		if(dot(hit.normal, ray.direction()) > 0.f)
			hit.normal = - hit.normal;
		auto target = hit.normal + random.unit_vector();
		out = math::Ray(hit.p, normalize(target));
		attenuation = albedo;
		return true;
	}

	math::Vec3f albedo;
};
