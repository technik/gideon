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

#include <vector>
#include <math/vector3.h>
#include "shape.h"
#include "triangle.h"

class TriangleMesh : public Shape
{
public:
	TriangleMesh() = default;

	template<typename Idx>
	TriangleMesh(
		const std::vector<math::Vec3f>& vertices,
		const std::vector<Idx>& indices)
	{
		auto nTris = indices.size() / 3;
		mTris.resize(nTris);
		for(size_t i = 0; i < nTris; ++i)
		{
			auto i0 = indices[3*i+0];
			auto i1 = indices[3*i+1];
			auto i2 = indices[3*i+2];
			mTris[i] = Triangle(vertices[i0], vertices[i1], vertices[i2]);
		}
	}

	// Bruteforce approach
	bool hit(const math::Ray & r, float tMin, float tMax, HitRecord & collision) const override
	{
		float t = tMax;
		bool hit_anything = false;
		HitRecord tmp_hit;
		for(auto& tri : mTris)
		{
			if(tri.hit(r,tMin,t,tmp_hit))
			{
				collision = tmp_hit;
				t = tmp_hit.t;
				hit_anything = true;
			}
		}

		return hit_anything;
	}

private:
	std::vector<Triangle> mTris;
};