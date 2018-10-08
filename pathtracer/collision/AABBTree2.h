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
#include <shapes/shape.h>
#include <shapes/triangle.h>
#include <vector>

/// Two way AABB tree
template<class Leaf>
class AABBTree2 final : public Shape
{
public:
	AABBTree2() = default;
	AABBTree2(std::vector<Leaf>& triangles);

	bool hit(
		const math::Ray& r,
		float tMax,
		HitRecord& collision
	) const override
	{
		// You should test against BBox before. And that should always return
		// fals when there are no nodes
		assert(mNodes.size() > 0);

		return hitSubtree(mNodes.front(), r, tMax, collision);
	}

private:

	struct Child {
		math::AABB mBbox;
		size_t mIndex;
		bool mIsLeaf;
	};

	struct Node {
		ChildInfo mChildren[2];
	};

private:
	static bool hitSubtree(
		const Node& root,
		const math::Ray& r,
		float tMax,
		HitRecord& collision);

private:
	std::vector<Node> mNodes;
	std::vector<Leaf> mLeafs;
};
