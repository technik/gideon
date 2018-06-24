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

#include <array>
#include <math/ray.h>
#include <math/vector.h>

class Triangle
{
public:
	Triangle() = default;
	Triangle(
		const math::Vec3f& v0,
		const math::Vec3f& v1,
		const math::Vec3f& v2)
		: v({v0,v1,v2})
	{
		edge0 = v[1]-v[0];
		edge1 = v[2]-v[1];
		edge2 = v[0]-v[2];
		mNormal = normalize(cross(edge0,edge1));
	}

	bool hit(
		const math::Ray& r,
		float tMin,
		float tMax,
		HitRecord& collision
	) const
	{
		auto p0 = r.at(tMin);
		auto a0 = cross(v[0]-p0, edge0);
		auto a1 = cross(v[1]-p0, edge1);
		auto a2 = cross(v[2]-p0, edge2);

		if((dot(a0,r.direction()) < 0.f) && (dot(a1,r.direction()) < 0.f) && (dot(a2,r.direction()) < 0.f))
		{
			float t = dot(r.direction(), v[0]);
			if( t >= tMin && t < tMax)
			{
				auto p = r.at(t);

				collision.t = t;
				collision.p = p;
				collision.normal = mNormal;

				return true;
			}
		}

		return false;
	}

	const math::Vec3f& vtx(size_t i) const
	{
		return v[i];
	}

	const math::Vec3f& normal() const {
		return mNormal;
	}
	math::Vec3f centroid() const { return (v[0]+v[1]+v[2])/3.f; }

//private:
	std::array<math::Vec3f,3> v;
	math::Vec3f edge0;
	math::Vec3f edge1;
	math::Vec3f edge2;
	math::Vec3f mNormal;
};
