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
	AABBTree2(std::vector<Leaf>& elements)
	{
		mLeafs = elements;
		// Construct tree
		mNodes.reserve(2*elements.size()-1);
		mNodes.resize(1);
		math::AABBSimd simdBBox;
		initSubtree(mNodes[0], mLeafs.begin(), mLeafs.end(), 0, simdBBox);
		auto min = math::Vec3f(simdBBox.min().x(), simdBBox.min().y(), simdBBox.min().z());
		auto max = math::Vec3f(simdBBox.max().x(), simdBBox.max().y(), simdBBox.max().z());
		mBBox = { min, max };
	}

	bool hit(
		const math::Ray& r,
		float tMax,
		HitRecord& collision
	) const override
	{
		// You should test against BBox before. And that should always return
		// false when there are no nodes
		assert(mNodes.size() > 0);

		return hitSubtree(mNodes.front(), r, tMax, collision);
	}

private:

	struct Child {
		math::AABBSimd mBBox;
		size_t mIndex;
		bool mIsLeaf;

		bool isLeaf() const { return mIsLeaf; }
	};

	struct Node {
		Child mChildren[2];
	};

private:
	void initSubtree(
		Node& root,
		typename std::vector<Leaf>::iterator triangleBegin,
		typename std::vector<Leaf>::iterator triangleEnd,
		unsigned sortAxis,
		math::AABBSimd& subtreeBBox
	);

	bool hitSubtree(
		const Node& root,
		const math::Ray& r,
		float tMax,
		HitRecord& collision) const;

	void initNodeChild(
		Child& child,
		typename std::vector<Leaf>::iterator elementsBegin,
		typename std::vector<Leaf>::iterator elementsEnd,
		unsigned nextAxis);
	void initLeafNodeChild(Child&, typename std::vector<Leaf>::iterator leaf);
	void initBranchNodeChild(
		Child& child,
		typename std::vector<Leaf>::iterator elementsBegin,
		typename std::vector<Leaf>::iterator elementsEnd,
		unsigned nextAxis);

private:
	std::vector<Node> mNodes;
	std::vector<Leaf> mLeafs;
};

//----------------------------------------------------------------------------------------
template<class Leaf>
void AABBTree2<Leaf>::initSubtree(
	Node& root,
	typename std::vector<Leaf>::iterator elementsBegin,
	typename std::vector<Leaf>::iterator elementsEnd,
	unsigned sortAxis,
	math::AABBSimd& subtreeBBox
){
	auto nElements = elementsEnd-elementsBegin;
	assert(nElements >= 2); // Otherwise, we should be initializing a leaf

	// Approximately sort triangles along the given axis
	math::Vec3f axis(0.f);
	axis[sortAxis] = 1.f;
	std::sort(elementsBegin, elementsEnd,
		[axis](Triangle& a, Triangle& b) -> bool {
		auto da = dot(a.centroid(), axis); 
		auto db = dot(b.centroid(), axis); 
		return da < db;
	});
	// TODO: Try a bunch of possible splits

	// Find the middle element, making sure the first half is always >= the second half
	// This is useful for nodes with one child branch and one child leaf, so the branch
	// Always takes the first place
	auto middle = elementsBegin+(nElements+1)/2;
	assert((middle - elementsBegin) >= (elementsEnd - middle));

	// Init node children
	auto nextAxis = (sortAxis+1)%3;
	initNodeChild(root.mChildren[0], elementsBegin, middle, nextAxis);
	initNodeChild(root.mChildren[1], middle, elementsEnd, nextAxis);

	// Update bbox
	subtreeBBox = math::AABBSimd(root.mChildren[0].mBBox, root.mChildren[1].mBBox);
}

//----------------------------------------------------------------------------------------
template<class Leaf>
bool AABBTree2<Leaf>::hitSubtree(
	const Node& root,
	const math::Ray& r,
	float tMax,
	HitRecord& collision) const
{
	float t = tMax;
	bool hit_anything = false;

	HitRecord tmp_hit;
	for (auto& child : root.mChildren)
	{
		if(child.mBBox.intersect(r.implicitSimd(), tMax))
		{
			if(child.isLeaf()) {
				if(mLeafs[child.mIndex].hit(r, t, tmp_hit)) {
					collision = tmp_hit;
					t = tmp_hit.t;
					hit_anything = true;
				}
			}
			else if(hitSubtree(mNodes[child.mIndex], r, t, tmp_hit)) {
					collision = tmp_hit;
					t = tmp_hit.t;
					hit_anything = true;
				}
		}
	}

	return hit_anything;
}

//----------------------------------------------------------------------------------------
template<class Leaf>
void AABBTree2<Leaf>::initNodeChild(
	Child& child,
	typename std::vector<Leaf>::iterator elementsBegin,
	typename std::vector<Leaf>::iterator elementsEnd,
	unsigned nextAxis)
{
	auto numElements = elementsEnd - elementsBegin;
	if(numElements > 1)
		initBranchNodeChild(child, elementsBegin, elementsEnd, nextAxis);
	else // Child A is a leaf
		initLeafNodeChild(child, elementsBegin);
}

//----------------------------------------------------------------------------------------
template<class Leaf>
void AABBTree2<Leaf>::initLeafNodeChild(
	Child& child,
	typename std::vector<Leaf>::iterator leafIter)
{
	auto& leaf = *leafIter;
	child.mBBox = math::AABBSimd(leaf.bbox().min(), leaf.bbox().max());
	child.mIndex = leafIter-mLeafs.begin();
	child.mIsLeaf = true;
}

//----------------------------------------------------------------------------------------
template<class Leaf>
void AABBTree2<Leaf>::initBranchNodeChild(
	Child& child,
	typename std::vector<Leaf>::iterator elementsBegin,
	typename std::vector<Leaf>::iterator elementsEnd,
	unsigned nextAxis)
{
	child.mIsLeaf = false;
	child.mIndex = mNodes.size();
	mNodes.push_back({});
	initSubtree(mNodes.back(), 
		elementsBegin, elementsEnd,
		nextAxis, child.mBBox);
}
