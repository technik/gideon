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
	AABBTree(const std::vector<Triangle>& triangles)
	{
		mNodes.reserve(2*triangles.size()-1);
		mNodes.resize(1);
		initNode(mNodes[0], triangles, 0);
	}

	bool hit(const math::Ray & r, const math::Ray::ImplicitSimd& ri, math::float4 tMin, math::float4 tMax, HitRecord & collision) const
	{
		auto hit_any = hitNode(mNodes[0], r, ri, tMin, tMax, collision);
		if(hit_any)
		{
			collision.p = r.at(collision.t);
		}
		return hit_any;
	}

	const math::AABBSimd& bbox() const { return mRoot.mBbox; }

private:

	struct Node
	{
		Node() {
			mBbox.clear();
		};

		bool isLeaf() const
		{
			return mChildA == mChildB;
		}

		math::AABBSimd mBbox;
		size_t mChildA, mChildB;
		std::vector<Triangle> mTriangles;
	};

	void initNode(Node& node, std::vector<Triangle> triangles, unsigned sortAxis)
	{
		if(triangles.size() > nMaxLeafElements) // To many elements, branch
		{
			// Approximately sort triangles along the given axis
			math::Vec3f axis(0.f);
			axis[sortAxis] = 1.f;
			std::sort(triangles.begin(), triangles.end(),
				[axis](Triangle& a, Triangle& b) -> bool {
				auto da = dot(a.centroid(), axis); 
				auto db = dot(b.centroid(), axis); 
				return da < db;
			});
			// Create children nodes
			auto middle = triangles.size() / 2;
			auto nextAxis = (sortAxis+1)%3;
			node.mChildA = mNodes.size();
			node.mChildB = node.mChildA+1;
			mNodes.resize(mNodes.size()+2);
			// Child A
			std::vector<Triangle> childTris;
			childTris.insert(childTris.begin(), triangles.begin(), triangles.begin()+middle);
			initNode(mNodes[node.mChildA], childTris, nextAxis);
			// Child B
			childTris.clear();
			childTris.insert(childTris.begin(), triangles.begin()+middle, triangles.end());
			initNode(mNodes[node.mChildB], childTris, nextAxis);

			// Update bbox
			node.mBbox = math::AABBSimd(mNodes[node.mChildA].mBbox, mNodes[node.mChildB].mBbox);
		}
		else // Leaf node
		{
			node.mChildA = node.mChildB = 0;
			node.mTriangles = std::move(triangles);
			AABB rawBBox;
			rawBBox.clear();
			for(auto& t : node.mTriangles)
			{
				rawBBox.add(t.v[0]);
				rawBBox.add(t.v[1]);
				rawBBox.add(t.v[2]);
			}
			node.mBbox = AABBSimd(rawBBox.min(), rawBBox.max());
		}
	}

	bool hitNode(const Node& node, const math::Ray& r, const math::Ray::ImplicitSimd& ri, math::float4 tMin, math::float4 tMax, HitRecord & collision) const
	{
		if(!node.isLeaf()) // Non-leaf
		{
			// Check children
			bool hit_any = false;
			auto& nodeA = mNodes[node.mChildA];
			if(nodeA.mBbox.intersect(ri, tMin,tMax,collision.t) && hitNode(nodeA,r,ri,tMin,tMax,collision))
			{
				tMax = float4(collision.t);
				hit_any = true;
			}
			auto& nodeB = mNodes[node.mChildB];
			if(nodeB.mBbox.intersect(ri, tMin,tMax,collision.t) && hitNode(nodeB,r,ri,tMin,tMax,collision))
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

			auto stMin = tMin.x();
			auto p0 = r.at(stMin);
			for(auto& triangle : node.mTriangles)
			{
				if(triangle.hit(r, stMin, tMax.x(), p0, collision))
				{
					hit_any = true;
					tMax= float4(collision.t);
				}
			}

			return hit_any;
		}
	}

	std::vector<Node> mNodes;
};
