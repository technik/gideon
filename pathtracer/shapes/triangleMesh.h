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

#include <cassert>
#include <vector>
#include <materials/material.h>
#include <math/vector.h>
#include <math/aabb.h>
#include <collision/AABBTree.h>
#include "shape.h"
#include "triangle.h"

//-------------------------------------------------------------------------------------------------
class TriangleMesh
{
public:
	TriangleMesh() = default;

	struct VtxInfo
	{
		math::Vec3f position;
		math::Vec3f normal;

		VtxInfo lerp(const VtxInfo& b, float x) const;
	};

	template<typename Idx>
	TriangleMesh(
		const std::vector<VtxInfo>& vertices,
		const std::vector<Idx>& indices,
        const std::shared_ptr<Material>& material);

	bool hit(const math::Ray & r, float tMax, HitRecord & collision) const;
    const auto& bbox() const { return mBBox; }

private:

	struct TriangleHit
	{
		float t;
		float u, v; // Barycentrics
	};

	void interpolateData(uint32_t triNdx, const TriangleHit& tri, HitRecord& hit) const
	{
		auto& v0 = mVtxData[3*triNdx];
		auto& v1 = mVtxData[3*triNdx+1];
		auto& v2 = mVtxData[3*triNdx+2];
		auto a = v0.lerp(v1,tri.u);
		auto res = a.lerp(v2,tri.v);
		
		hit.normal = res.normal;
		normalize(hit.normal);
	}

    math::AABB mBBox; // Bounding box
	AABBTree<2> mBVH;
	std::vector<uint16_t> mIndices;
	std::vector<VtxInfo> mVtxData;
    std::shared_ptr<Material> mMaterial;
};

//-------------------------------------------------------------------------------------------------
// Inline implementation
//-------------------------------------------------------------------------------------------------
inline TriangleMesh::VtxInfo TriangleMesh::VtxInfo::lerp(const VtxInfo& b, float x) const
{ 
	VtxInfo res;
	auto x0 = 1-x;
	res.position = position*x0+b.position*x;
	res.normal = math::lerp(normal,b.normal, x);
	return res;
}

//-------------------------------------------------------------------------------------------------
template<typename Idx>
TriangleMesh::TriangleMesh(
	const std::vector<VtxInfo>& vertices,
	const std::vector<Idx>& indices,
    const std::shared_ptr<Material>& material)
    : mMaterial(material)
{
	mVtxData = vertices;
	auto nTris = indices.size() / 3;
	mIndices.resize(indices.size());

	std::vector<Triangle> triangles(nTris);
	for(auto i = 0; i < nTris; ++i)
	{
		auto i0 = indices[3*i+0];
		auto i1 = indices[3*i+1];
		auto i2 = indices[3*i+2];
		mIndices[3*i+0] = i0;
		mIndices[3*i+1] = i1;
		mIndices[3*i+2] = i2;
		// Construct the triangle
		triangles[i] = Triangle(mVtxData[i0].position, mVtxData[i1].position, mVtxData[i2].position);
	}

	mBBox.clear();
	for(auto& v : vertices)
		mBBox.add(v.position);

	mBVH = AABBTree<2>(triangles);
}

//-------------------------------------------------------------------------------------------------
inline bool TriangleMesh::hit(const math::Ray & r, float tMax, HitRecord & collision) const
{
    if (mBVH.hit(r, r.implicitSimd(), math::float4(tMax), collision))
    {
        return true;
    }
    return false;
}