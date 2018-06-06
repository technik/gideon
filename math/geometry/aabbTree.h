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
#include <shapes/triangle.h>
#include <shapes/triangleSet.h>

namespace math {

	struct AABBTree {

		using TriList = std::vector<TriInfo>;
		using TriRange = std::pair<TriList::iterator, TriList::iterator>;
		static constexpr size_t MAX_LEAF_TRIS = 2;
		using TriSet = TriangleSet<MAX_LEAF_TRIS>;

		AABBTree() = default;
		AABBTree(std::vector<TriInfo>& triList)
		{
			TriRange range = {triList.begin(), triList.end()};
			mNumElements = triList.size();
			prepareTreeRange(range);
		}

		// Returns index of the bounding volume enclosing the range
		size_t prepareTreeRange(const TriRange& range, int partitionAxis = 0);

		using Children = std::pair<size_t,size_t>;

		struct Node
		{
			math::AABBSimd bbox;
			Children children;

			bool isLeaf() const { 
				return children.first == -1;
			}

			void maskAsLeaf()
			{
				children.first = -1;
			}
		};

		std::vector<Node> mNodes;
		std::vector<TriSet> mTriSets;

		size_t mNumElements;


		bool hit(size_t ndx, uint32_t rangeLen, const math::Ray & r, const math::Ray::ImplicitSimd & ri, float tMin, float tMax, TriangleHit & collision) const;

		bool hit(const math::Ray & r, float tMin, float tMax, TriangleHit & collision) const
		{
			return hit(0, mNumElements, r, r.implicitSimd() ,tMin,tMax,collision);
		}
	};

	//-------------------------------------------------------------------------------------------------
	inline size_t AABBTree::prepareTreeRange(const TriRange& range, int partitionAxis)
	{
		size_t ndx = mNodes.size();
		mNodes.emplace_back();
		auto nElements = range.second-range.first;
		if(nElements <= MAX_LEAF_TRIS) // leaf node
		{
			auto& bbox = mNodes.back().bbox;
			bbox.clear();
			for(auto t = range.first; t != range.second; ++t)
			{
				bbox.add(math::float4(t->tri.vtx(0)));
				bbox.add(math::float4(t->tri.vtx(1)));
				bbox.add(math::float4(t->tri.vtx(2)));
			}
			mNodes.back().maskAsLeaf(); // Mark node as leaf
			mNodes.back().children.second = mTriSets.size();
			mTriSets.emplace_back(range);
		}
		else // Non-leaf node
		{
			math::Vec3f axis(0.f);
			axis[partitionAxis] = 1.f;
			auto sortedList = range;
			std::sort(sortedList.first, sortedList.second, 
				[axis](const TriInfo& a, const TriInfo& b) {
				return dot(a.tri.centroid()-b.tri.centroid(), axis) < 0.f;
			});
			auto middle = sortedList.first + lower_power_of_two(nElements);
			auto a = prepareTreeRange({sortedList.first, middle}, (partitionAxis+1)%3);
			auto b = prepareTreeRange({middle, sortedList.second }, (partitionAxis+1)%3);
			mNodes[ndx] = {
				math::AABBSimd(mNodes[a].bbox, mNodes[b].bbox),
			{a,b}
			};
		}
		return ndx;
	}

	//--------------------------------------------------------------------------------------------------
	inline bool AABBTree::hit(size_t ndx, uint32_t rangeLen, const math::Ray & r, const math::Ray::ImplicitSimd & ri, float tMin, float tMax, TriangleHit & collision) const
	{
		auto& node = mNodes[ndx];
		if(!node.bbox.intersect(ri, tMin, tMax, tMin))
			return false;

		float t = tMax;
		TriangleHit tmp_hit;
		if(!node.isLeaf()) // this is not a leaf node
		{
			auto middle = lower_power_of_two(rangeLen);
			auto& children = node.children;
			bool hit_a = hit(children.first, middle,r,ri,tMin,t,tmp_hit);
			if(hit_a)
			{
				collision = tmp_hit;
				t = tmp_hit.t;
			}
			bool hit_b = hit(children.second, rangeLen-middle,r,ri,tMin,t,tmp_hit);
			if(hit_b)
			{
				collision = tmp_hit;
				t = tmp_hit.t;
			}
			return hit_a || hit_b;
		}
		else
		{
			HitRecord tri_hit;
			float f0, f1;
			auto& set = mTriSets[node.children.second];
			auto i = set.hit(r,tMin,tMax,tri_hit,f0,f1);
			if(i >= 0)
			{
				collision.pos = tri_hit.p;
				t = tri_hit.t;
				collision.t = t;
				collision.f0 = f0;
				collision.f1 = f1;
				collision.normal = tri_hit.normal;
				collision.ndx = set.indices[i];
				return true;
			}

			return false;
		}
	}

}
