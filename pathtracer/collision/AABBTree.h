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
		mRoot = Node(triangles, 0);
	}

	bool hit(const math::Ray & r, const math::Ray::ImplicitSimd& ri, math::float4 tMin, math::float4 tMax, HitRecord & collision) const
	{
		return mRoot.hit(r, ri, tMin, tMax, collision);
	}

	const math::AABBSimd& bbox() const { return mRoot.mBbox; }

private:
	struct Node
	{
		Node() {
			mBbox.clear();
		};
		Node(std::vector<Triangle> triangles, unsigned sortAxis)
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
				// Child A
				std::vector<Triangle> childTris;
				childTris.insert(childTris.begin(), triangles.begin(), triangles.begin()+middle);
				mChildren.emplace_back(childTris, nextAxis);
				// Child B
				childTris.clear();
				childTris.insert(childTris.begin(), triangles.begin()+middle, triangles.end());
				mChildren.emplace_back(childTris, nextAxis);

				// Update bbox
				mBbox = math::AABBSimd(mChildren[0].mBbox, mChildren[1].mBbox);
			}
			else // Leaf node
			{
				mTriangles = std::move(triangles);
				AABB rawBBox;
				rawBBox.clear();
				for(auto& t : mTriangles)
				{
					rawBBox.add(t.v[0]);
					rawBBox.add(t.v[1]);
					rawBBox.add(t.v[2]);
				}
				mBbox = AABBSimd(rawBBox.min(), rawBBox.max());
			}
		}

		bool hit(const math::Ray& r, const math::Ray::ImplicitSimd& ri, math::float4 tMin, math::float4 tMax, HitRecord & collision) const
		{
			if(!mChildren.empty()) // Non-leaf
			{
				// Check children
				bool hit_any = false;
				if(mChildren[0].mBbox.intersect(ri, tMin,tMax,collision.t) && mChildren[0].hit(r,ri,tMin,tMax,collision))
				{
					tMax = float4(collision.t);
					hit_any = true;
				}
				if(mChildren[1].mBbox.intersect(ri, tMin,tMax,collision.t) && mChildren[1].hit(r,ri,tMin,tMax,collision))
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

				for(auto& triangle : mTriangles)
				{
					if(triangle.hit(r, tMax.x(), collision))
					{
						hit_any = true;
						tMax= float4(collision.t);
					}
				}

				return hit_any;
			}
		}

		math::AABBSimd mBbox;
		std::vector<Node> mChildren;
		std::vector<Triangle> mTriangles;
	};

	Node mRoot;
};
