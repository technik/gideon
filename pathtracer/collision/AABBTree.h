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
#include <shapes/triangle.h>
#include <vector>

template<
	size_t nMaxLeafElements ///< Max number of elements in a leaf node
> 
class AABBTree
{
public:
	AABBTree(const std::vector<Triangle>& triangles)
	{
		root = new Node(triangles, 0);
	}

	bool hit(const math::Ray & r, float tMin, float tMax, HitRecord & collision) const
	{
		return root.hit(r, tMin, tMax, collision);
	}

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
				auto middle = triangles.size / 2;
				// Child A
				std::vector<Triangle> childTris;
				childTris.insert(childTris.begin(), triangles.begin(), triangles.begin()+middle);
				mChildren.emplace_back(childTris);
				// Child B
				childTris.clear();
				childTris.insert(childTris.begin(), triangles.begin()+middle, triangles.end());
				mChildren.emplace_back(childTris);

				// Update bbox
				mBbox = math::AABB(mChildren[0].mBbox, mChildren[1].mBbox);
			}
			else // Leaf node
			{
				mBbox.clear();
				mTriangles = std::move(triangles);
				for(auto& t : mTriangles)
				{
					mBbox.add(t.v[0]);
					mBbox.add(t.v[1]);
					mBbox.add(t.v[2]);
				}
			}
		}

		bool hit(const math::Ray & r, float tMin, float tMax, HitRecord & collision) const
		{
			if(!mChildren.empty()) // Non-leaf
			{
				// Check children
				bool hit_any = false;
				if(mChildren[0].hit(r,tMin,tMax,collision))
				{
					tMax = collision.t;
					hit_any = true;
				}
				if(mChildren[1].hit(r,tMin,tMax,collision))
				{
					tMax = collision.t;
					hit_any = true;
				}
				collision.t = tMax;
				return hit_any;
			}
			else // leaf node, check all triangles
			{
				bool hit_any = false;

				for(auto& triangle : mTriangles)
				{
					float f0, f1;
					if(triangle.hit(r, tMin, tMax, collision, f0, f1))
					{
						hit_any = true;
						tMax= collision.t;
					}
				}

				return hit_any;
			}
		}

		math::AABB mBbox;
		std::vector<Node> mChildren;
		std::vector<Triangle> mTriangles;
	};

	Node mRoot;
};
