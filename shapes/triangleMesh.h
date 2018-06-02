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
#include <material.h>
#include <math/vector3.h>
#include <math/aabb.h>
#include "shape.h"
#include "triangle.h"

//-------------------------------------------------------------------------------------------------
class TriangleMesh : public Shape
{
public:
	TriangleMesh() = default;

	struct VtxInfo
	{
		math::Vec3f position;
		math::Vec3f normal;
		float u, v;

		VtxInfo lerp(const VtxInfo& b, float x) const;
	};

	template<typename Idx>
	TriangleMesh(
		const std::vector<VtxInfo>& vertices,
		const std::vector<Idx>& indices);

	bool hit(const math::Ray & r, float tMin, float tMax, HitRecord & collision) const override;

	static constexpr size_t MAX_LEAF_TRIS = 8;

private:
	struct TriInfo {
		Triangle tri;
		int ndx;
	};

	struct TriangleHit
	{
		size_t ndx;
		math::Vec3f pos;
		math::Vec3f normal;
		float t;
		float f0, f1; // Interpolation factors
	};

	void interpolateData(const TriangleHit& tri, HitRecord& hit) const
	{
		auto& v0 = mVtxData[3*tri.ndx];
		auto& v1 = mVtxData[3*tri.ndx+1];
		auto& v2 = mVtxData[3*tri.ndx+2];
		auto a = v0.lerp(v1,tri.f0);
		auto res = a.lerp(v2,tri.f1);
		
		hit.u = res.u;
		hit.v = res.v;
		hit.normal = res.normal;
		hit.normal.normalize();
	}

	using TriList = std::vector<TriInfo>;
	using TriRange = std::pair<TriList::iterator, TriList::iterator>;

	struct AABBTree {
		AABBTree() = default;
		AABBTree(const std::vector<TriInfo>& triList)
		{
			mTris = triList;
			mRange = {mTris.begin(), mTris.end()};
			prepareTreeRange(mRange);
		}

		// Returns index of the bounding volume enclosing the range
		size_t prepareTreeRange(const TriRange& range, int partitionAxis = 0);

		using Children = std::pair<size_t,size_t>;

		struct Node
		{
			math::AABB bbox;
			Children children;
		};

		std::vector<Node> mNodes;

		TriRange mRange;
		std::vector<TriInfo> mTris;


		bool hit(size_t ndx, const TriRange& range, const math::Ray & r, const math::Ray::Implicit & ri, float tMin, float tMax, TriangleHit & collision) const;

		bool hit(const math::Ray & r, float tMin, float tMax, TriangleHit & collision) const
		{
			return hit(0, mRange, r, r.implicit() ,tMin,tMax,collision);
		}
	};

	AABBTree mBVH;
	std::vector<VtxInfo> mVtxData;
};

//-------------------------------------------------------------------------------------------------
// Inline implementation
//-------------------------------------------------------------------------------------------------
TriangleMesh::VtxInfo TriangleMesh::VtxInfo::lerp(const VtxInfo& b, float x) const
{ 
	VtxInfo res;
	auto x0 = 1-x;
	res.position = position*x0+b.position*x;
	res.normal = math::lerp(normal,b.normal, x);
	res.u = u*x0+b.u*x;
	res.v = v*x0+b.v*x;
	return res;
}

//-------------------------------------------------------------------------------------------------
template<typename Idx>
TriangleMesh::TriangleMesh(
	const std::vector<VtxInfo>& vertices,
	const std::vector<Idx>& indices)
{
	auto nTris = indices.size() / 3;
	std::vector<TriInfo> triList(nTris);
	mVtxData.resize(3*nTris);

	for(size_t i = 0; i < nTris; ++i)
	{
		auto i0 = indices[3*i+0];
		auto i1 = indices[3*i+1];
		auto i2 = indices[3*i+2];
		auto* vtx = &mVtxData[3*i];
		vtx[0] = vertices[i0];
		vtx[1] = vertices[i1];
		vtx[2] = vertices[i2];
		triList[i].ndx = i;
		triList[i].tri = Triangle(vtx[0].position, vtx[1].position, vtx[2].position);
	}

	mBVH = AABBTree(triList);
}

//-------------------------------------------------------------------------------------------------
inline bool TriangleMesh::hit(const math::Ray & r, float tMin, float tMax, HitRecord & collision) const
{
	TriangleHit hitInfo;
	if(mBVH.hit(r,tMin,tMax,hitInfo))
	{
		collision.p = hitInfo.pos;
		collision.t = hitInfo.t;
		interpolateData(hitInfo, collision);
		return true;
	}
	return false;
}

//-------------------------------------------------------------------------------------------------
inline size_t TriangleMesh::AABBTree::prepareTreeRange(const TriRange& range, int partitionAxis)
{
	size_t ndx = mNodes.size();
	mNodes.emplace_back();
	if((range.second-range.first) <= TriangleMesh::MAX_LEAF_TRIS) // leaf node
	{
		auto& bbox = mNodes.back().bbox;
		bbox.clear();
		for(auto t = range.first; t != range.second; ++t)
		{
			bbox.add(t->tri.vtx(0));
			bbox.add(t->tri.vtx(1));
			bbox.add(t->tri.vtx(2));
		}
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
		auto middle = sortedList.first + (sortedList.second-sortedList.first) / 2;
		auto a = prepareTreeRange({sortedList.first, middle}, (partitionAxis+1)%3);
		auto b = prepareTreeRange({middle, sortedList.second }, (partitionAxis+1)%3);
		mNodes[ndx] = {
			math::AABB(mNodes[a].bbox, mNodes[b].bbox),
			{a,b}
		};
	}
	return ndx;
}

//--------------------------------------------------------------------------------------------------
inline bool TriangleMesh::AABBTree::hit(size_t ndx, const TriRange& range, const math::Ray & r, const math::Ray::Implicit & ri, float tMin, float tMax, TriangleHit & collision) const
{
	if(!mNodes[ndx].bbox.intersect(ri, tMin, tMax, tMin))
		return false;

	float t = tMax;
	TriangleHit tmp_hit;
	auto rangeLen = (range.second-range.first);
	if(rangeLen > MAX_LEAF_TRIS) // this is not a leaf node
	{
		auto middle = range.first + rangeLen/2;
		auto& children = mNodes[ndx].children;
		bool hit_a = hit(children.first, {range.first,middle},r,ri,tMin,t,tmp_hit);
		if(hit_a)
		{
			collision = tmp_hit;
			t = tmp_hit.t;
		}
		bool hit_b = hit(children.second, {middle,range.second},r,ri,tMin,t,tmp_hit);
		if(hit_b)
		{
			collision = tmp_hit;
			t = tmp_hit.t;
		}
		return hit_a || hit_b;
	}
	else
	{
		bool hit_anything = false;
		for(auto tri = range.first; tri != range.second; ++tri)
		{
			HitRecord tri_hit;
			float f0, f1;
			if(tri->tri.hit(r,tMin,t,tri_hit,f0,f1))
			{
				collision.pos = tri_hit.p;
				t = tri_hit.t;
				collision.t = t;
				collision.f0 = f0;
				collision.f1 = f1;
				collision.normal = tri_hit.normal;
				collision.ndx = tri->ndx;
				hit_anything = true;
			}
		}

		return hit_anything;
	}
}