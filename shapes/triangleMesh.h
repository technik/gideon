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

	static constexpr size_t MAX_LEAF_TRIS = 2;

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
		AABBTree(std::vector<TriInfo>& triList)
		{
			TriRange range = {triList.begin(), triList.end()};
			mNumElements = triList.size();
			prepareTreeRange(range);
		}

		// Returns index of the bounding volume enclosing the range
		size_t prepareTreeRange(const TriRange& range, int partitionAxis = 0);

		using Children = std::pair<size_t,size_t>;

		template<size_t N>
		struct TriSet
		{
			TriSet() = default;
			TriSet(const TriRange& range)
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

		struct Node
		{
			math::AABB bbox;
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
		std::vector<TriSet<MAX_LEAF_TRIS>> mTriSets;

		size_t mNumElements;


		bool hit(size_t ndx, size_t rangeLen, const math::Ray & r, const math::Ray::Implicit & ri, float tMin, float tMax, TriangleHit & collision) const;

		bool hit(const math::Ray & r, float tMin, float tMax, TriangleHit & collision) const
		{
			return hit(0, mNumElements, r, r.implicit() ,tMin,tMax,collision);
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
inline bool TriangleMesh::AABBTree::hit(size_t ndx, size_t rangeLen, const math::Ray & r, const math::Ray::Implicit & ri, float tMin, float tMax, TriangleHit & collision) const
{
	auto& node = mNodes[ndx];
	if(!node.bbox.intersect(ri, tMin, tMax, tMin))
		return false;

	float t = tMax;
	TriangleHit tmp_hit;
	if(!node.isLeaf()) // this is not a leaf node
	{
		auto middle = rangeLen/2;
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

//--------------------------------------------------------------------------------------------------
template<size_t N>
inline int TriangleMesh::AABBTree::TriSet<N>::hit(
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