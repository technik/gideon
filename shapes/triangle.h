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
#include <math/vector3.h>

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
		mNormal = normalize(cross(edge0,edge1));

		mPlaneOffset = dot(v[0],mNormal);
	}

	bool hit(
		const math::Ray& r,
		float tMin,
		float tMax,
		HitRecord& collision,
		float& f0, float& f1 // Interpolation factors along the edges
	) const
	{
		auto p0 = r.at(tMin);
		auto p1 = r.at(tMax);

		auto offset0 = dot(p0, mNormal);
		auto offset1 = dot(p1, mNormal);

		if((offset0-mPlaneOffset)*(offset1-mPlaneOffset) <= 0.f) // Line segment intersects the plane of the triangle
		{
			float t = tMin + (tMax-tMin)*(mPlaneOffset-offset0)/(offset1-offset0);
			auto p = r.at(t);

			auto c0 = cross(edge0,p-v[0]);
			auto c1 = cross(edge1,p-v[1]);
			if(dot(c0,c1) >= 0.f)
			{
				auto edge2 = v[0]-v[2];
				auto c2 = cross(edge2,p-v[2]);
				if(dot(c1,c2) >= 0.f)
				{
					// Get an orthogonal basis in the triangle
					auto e1ProjE0 = dot(edge1,edge0)/edge0.sqNorm();// Horizontal projection of edge1 over edge 0
					auto orthoE1 = edge1 - e1ProjE0*edge0; // V2' to V2
					f1 = 1 - dot(orthoE1,v[2]-p)/orthoE1.sqNorm();
					if(f1 >= 1.f-1e-6f)
						f0 = 0.f;
					else
					{
						auto pp = v[2] + (p-v[2])/(1.f-f1);
						f0 = (pp-v[0]).norm() / edge0.norm();
					}

					collision.t = t;
					collision.p = p;
					collision.normal = mNormal;
					return true;
				}
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

private:
	std::array<math::Vec3f,3> v;
	math::Vec3f edge0;
	math::Vec3f edge1;
	math::Vec3f mNormal;
	float mPlaneOffset;
};
