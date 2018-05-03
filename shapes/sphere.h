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

#include <material.h>
#include <math/ray.h>
#include <math/vector3.h>
#include "shape.h"

class Sphere : public Shape
{
public:
	Sphere(){}
	Sphere(const math::Vec3f& center, float radius, Material* mat)
		: mCenter(center)
		, mSqRadius(radius*radius)
		, m(mat)
	{}

	bool hit(
		const math::Ray& r,
		float tMin,
		float tMax,
		HitRecord& collision
	) const override
	{
		auto ro = r.origin() - mCenter; // Ray origin relative to sphere's center
		float a = r.direction().sqNorm();
		float b = dot(ro, r.direction());
		float c = ro.sqNorm() - mSqRadius;
		auto discriminant = b*b-a*c;
		if(discriminant >= 0)
		{
			float t = (-b - sqrt(discriminant)) / a;
			if(t > tMin && t < tMax) {
				collision.t = t;
				collision.p = r.at(t);
				collision.normal = normalize(collision.p - mCenter);
				collision.material = m;
				return true;
			}
		}
		return false;
	}

private:
	math::Vec3f mCenter;
	float mSqRadius;
	Material* m;
};
