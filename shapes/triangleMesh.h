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

#include <cassert>
#include <vector>
#include <math/vector3.h>
#include <math/aabb.h>
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
		std::vector<Triangle> triList(nTris);

		for(size_t i = 0; i < nTris; ++i)
		{
			auto i0 = indices[3*i+0];
			auto i1 = indices[3*i+1];
			auto i2 = indices[3*i+2];
			triList[i] = Triangle(vertices[i0], vertices[i1], vertices[i2]);
		}

		mBVH = AABBTree(triList);
	}

	bool hit(const math::Ray & r, float tMin, float tMax, HitRecord & collision) const override
	{
		return mBVH.hit(r,tMin,tMax,collision);
	}

private:
	struct AABBTree
	{
		math::AABB mBoundingVolume;
		AABBTree* a = nullptr;
		AABBTree* b = nullptr;
		std::vector<Triangle> mTris;

		AABBTree() = default;

		//--------------------------------------------------------------------------------------------------
		AABBTree(const std::vector<Triangle>& triangleList, int partitionAxis = 0)
		{
			if(triangleList.size() <= 8)
			{
				mTris = triangleList;
				mBoundingVolume.clear();
				for(auto& t : mTris)
				{
					mBoundingVolume.add(t.v[0]);
					mBoundingVolume.add(t.v[1]);
					mBoundingVolume.add(t.v[2]);
				}
			}
			else
			{
				math::Vec3f axis(0.f);
				axis[partitionAxis] = 1.f;
				std::vector<Triangle> sortedList = triangleList;
				std::sort(sortedList.begin(), sortedList.end(), 
					[axis](const Triangle& a, const Triangle& b) {
						return dot(a.centroid()-b.centroid(), axis) < 0.f;
				});
				auto middle = triangleList.size() / 2;
				a = new AABBTree(std::vector<Triangle>(triangleList.begin(), triangleList.begin() + middle), (partitionAxis+1)%3);
				b = new AABBTree(std::vector<Triangle>(triangleList.begin() + middle, triangleList.end()), (partitionAxis+1)%3);
				mBoundingVolume = math::AABB(a->mBoundingVolume, b->mBoundingVolume);
			}
		}

		//--------------------------------------------------------------------------------------------------
		bool hit(const math::Ray & r, float tMin, float tMax, HitRecord & collision) const
		{
			if(!mBoundingVolume.intersect(r.implicit(), tMin, tMax, tMax))
				return false;

			if(mTris.empty())
			{
				assert(a && b);
				float t = tMax;
				// Bruteforce approach
				bool hit_anything = false;
				HitRecord tmp_hit;
				bool hit_a = a->hit(r,tMin,t,tmp_hit);
				if(hit_a)
				{
					collision = tmp_hit;
					t = tmp_hit.t;
				}
				bool hit_b = b->hit(r,tMin,t,tmp_hit);
				if(hit_b)
				{
					collision = tmp_hit;
					t = tmp_hit.t;
				}
				return hit_a || hit_b;
			}
			else
			{
				assert(!a && !b);
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
		}
	};

	AABBTree mBVH;
};