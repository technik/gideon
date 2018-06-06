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

#include <math/vector3.h>

template<size_t N>
struct TriangleSet
{
	TriangleSet() = default;
	TriangleSet(const TriRange& range);

	math::Vec3f mV0[N];
	math::Vec3f mV1[N];
	math::Vec3f mV2[N];
	math::Vec3f mEdge0[N];
	math::Vec3f mEdge1[N];
	math::Vec3f mNormal[N];
	float mPlaneOffset[N];
	int indices[N];

	int hit(
		const math::Ray& r,
		float tMin,
		float tMax,
		HitRecord& collision,
		float& f0, float& f1 // Interpolation factors along the edges
	) const;
};

//--------------------------------------------------------------------------------------------------
// Inline implementation
//--------------------------------------------------------------------------------------------------
template<size_t N>
TriangleSet<N>::TriangleSet(const TriRange& range)
{
	for(int i = 0; i < N; ++i)
	{
		// Fill in with invalid trianlges
		mPlaneOffset[i] = std::numeric_limits<float>::infinity();
		indices[i] = -1;
	}
	for(int i = 0; i < N; ++i)
	{
		auto v =  (range.first+i)->tri.v;
		mV0[i] = v[0];
		mV1[i] = v[1];
		mV2[i] = v[2];
		mEdge0[i] =  (range.first+i)->tri.edge0;
		mEdge1[i] =  (range.first+i)->tri.edge1;
		mNormal[i] =  (range.first+i)->tri.mNormal;
		mPlaneOffset[i] =  (range.first+i)->tri.mPlaneOffset;
		indices[i] = (range.first+i)->ndx;
	}
}

//--------------------------------------------------------------------------------------------------
template<size_t N>
int TriangleSet<N>::hit(
	const math::Ray& r,
	float tMin,
	float tMax,
	HitRecord& collision,
	float& _f0, float& _f1 // Interpolation factors along the edges
) const
{
	auto p0 = r.at(tMin);
	auto p1 = r.at(tMax);

	int hit_anything = -1;

	collision.t = tMax;

	for(auto i = 0; i < N; ++i)
	{
		auto normal = mNormal[i];
		float offset0 = dot(p0, normal);
		float offset1 = dot(p1, normal);
		auto planeOffset = mPlaneOffset[i];

		if((offset0-planeOffset)*(offset1-planeOffset) <= 0.f) // Line segment intersects the plane of the triangle
		{
			float t = tMin + (tMax-tMin)*(planeOffset-offset0)/(offset1-offset0);
			auto p = r.at(t);

			auto edge0 = mEdge0[i];
			auto edge1 = mEdge1[i];
			auto v0 = mV0[i];
			auto v1 = mV1[i];
			auto c0 = cross(edge0,p-v0);
			auto c1 = cross(edge1,p-v1);
			if(dot(c0,c1) >= 0.f)
			{
				auto v2 = mV2[i];
				auto edge2 = v0-v2;
				auto c2 = cross(edge2,p-v2);
				if(dot(c1,c2) >= 0.f)
				{
					// Get an orthogonal basis in the triangle
					auto e1ProjE0 = dot(edge1,edge0)/edge0.sqNorm();// Horizontal projection of edge1 over edge 0
					auto orthoE1 = edge1 - e1ProjE0*edge0; // V2' to V2
					float f1 = 1 - dot(orthoE1,v2-p)/orthoE1.sqNorm();
					float f0;
					if(f1 >= 1.f-1e-6f)
						f0 = 0.f;
					else
					{
						auto pp = v2 + (p-v2)/(1.f-f1);
						f0 = (pp-v0).norm() / edge0.norm();
					}

					if(t < collision.t) {
						_f0 = f0;
						_f1 = f1;
						collision.t = t;
						collision.p = p;
						collision.normal = normal;
						hit_anything = i;
					}
				}
			}
		}
	}

	return hit_anything;
}
