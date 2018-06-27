//-------------------------------------------------------------------------------------------------
// Toy path tracer
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

#include <math/aabb.h>
#include <math/vector.h>
#include <math/vectorFloat.h>
#include <shapes/triangle.h>
#include <vector>

template<
	size_t nMaxLeafElements ///< Max number of elements in a leaf node
> 
class AABBTree
{
public:
	AABBTree() = default;
	AABBTree(std::vector<Triangle>& triangles)
	{
		// Construct tree
		mNodes.reserve(2*triangles.size()-1);
		mNodes.resize(1);
		initNode(mNodes[0], triangles.begin(), triangles.end(), triangles, 0);
		// Create simd versions of the triangles
		mTriangles.reserve(triangles.size());
		for(auto& t : triangles)
			mTriangles.emplace_back(t.simd());
	}

	bool hit(const math::Ray & r, const math::Ray::ImplicitSimd& ri, math::float4 tMax, HitRecord & collision) const
	{
		return hitNode(mNodes[0], r, ri, tMax, collision);
	}

	const math::AABBSimd& bbox() const { return mRoot.mBbox; }

private:

	struct Node
	{
		Node() {
			mBbox.clear();
		};

		math::AABBSimd mBbox;
		size_t mChildA, mChildB;
		bool isLeaf = false;
	};

	void initNode(
		Node& node,
		std::vector<Triangle>::iterator triangleBegin,
		std::vector<Triangle>::iterator triangleEnd,
		std::vector<Triangle>& triangles,
		unsigned sortAxis)
	{
		auto nTris = triangleEnd-triangleBegin;
		if(nTris > nMaxLeafElements) // To many elements, branch
		{
			// Approximately sort triangles along the given axis
			math::Vec3f axis(0.f);
			axis[sortAxis] = 1.f;
			std::sort(triangleBegin, triangleEnd,
				[axis](Triangle& a, Triangle& b) -> bool {
				auto da = dot(a.centroid(), axis); 
				auto db = dot(b.centroid(), axis); 
				return da < db;
			});
			// Try a bunch of possible splits
			auto middle = triangleBegin+nTris/2;

			// Create children nodes
			auto nextAxis = (sortAxis+1)%3;
			node.mChildA = mNodes.size();
			node.mChildB = node.mChildA+1;
			mNodes.resize(mNodes.size()+2);
			// Child A
			initNode(mNodes[node.mChildA], triangleBegin, middle, triangles, nextAxis);
			// Child B
			initNode(mNodes[node.mChildB], middle, triangleEnd, triangles, nextAxis);

			// Update bbox
			node.mBbox = math::AABBSimd(mNodes[node.mChildA].mBbox, mNodes[node.mChildB].mBbox);
			return;
		}

		node.isLeaf = true;
		node.mChildA = triangleBegin-triangles.begin();
		node.mChildB = triangleEnd-triangles.begin();
		AABB rawBBox = triangleRangeBounds(triangleBegin, triangleEnd);
		node.mBbox = AABBSimd(rawBBox.min(), rawBBox.max());
	}

	math::AABB triangleRangeBounds(
		std::vector<Triangle>::iterator triangleBegin,
		std::vector<Triangle>::iterator triangleEnd)
	{
		math::AABB bounds;
		bounds.clear();
		for(auto t = triangleBegin; t != triangleEnd; ++t)
		{
			bounds.add(t->v[0]);
			bounds.add(t->v[1]);
			bounds.add(t->v[2]);
		}
		return bounds;
	}

	math::AABB triangleBounds(const Triangle& t)
	{
		math::AABB bounds(
			math::min(t.v[0],t.v[1])
			math::max(t.v[1],t.v[2])
		);
		bounds.add(t.v[2]);
		return bounds;
	}

	bool hitNode(const Node& node, const math::Ray& r, const math::Ray::ImplicitSimd& ri, math::float4 tMax, HitRecord & collision) const
	{
		if(!node.isLeaf) // Non-leaf
		{
			// Check children
			bool hit_any = false;
			auto& nodeA = mNodes[node.mChildA];
			if(nodeA.mBbox.intersect(ri,tMax,collision.t) && hitNode(nodeA,r,ri,tMax,collision))
			{
				tMax = float4(collision.t);
				hit_any = true;
			}
			auto& nodeB = mNodes[node.mChildB];
			if(nodeB.mBbox.intersect(ri,tMax,collision.t) && hitNode(nodeB,r,ri,tMax,collision))
			{
				tMax = float4(collision.t);
				hit_any = true;
			}
			collision.t = tMax.x();
			return hit_any;
		}
		else // leaf node, check all triangles
		{
			bool hit_any = false;
			auto simdRay = r.simd();
			for(auto i = node.mChildA; i != node.mChildB; ++i)
			{
				if(mTriangles[i].hit(simdRay, tMax.x(), collision))
				{
					hit_any = true;
					tMax= float4(collision.t);
				}
			}

			return hit_any;
		}
	}

	std::vector<Node> mNodes;
	std::vector<Triangle::Simd> mTriangles;
};
