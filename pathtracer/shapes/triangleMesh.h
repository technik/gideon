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
#include <math/vector.h>
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
		math::Vec2f uv;

		VtxInfo lerp(const VtxInfo& b, float x) const;
	};

	template<typename Idx>
	TriangleMesh(
		const std::vector<VtxInfo>& vertices,
		const std::vector<Idx>& indices);

	bool hit(const math::Ray & r, float tMin, float tMax, HitRecord & collision) const override;

private:

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
		
		hit.uv = res.uv;
		hit.normal = res.normal;
		normalize(hit.normal);
	}

	std::vector<uint16_t> mIndices;
	std::vector<VtxInfo> mVtxData;
};

class MultiMesh : public Shape
{
public:
	MultiMesh(const std::vector<TriangleMesh>& mesh, const std::vector<std::shared_ptr<Material>>& mat)
		: mMeshes(mesh)
		, mMaterials(mat)
	{}

	bool hit(const math::Ray & r, float tMin, float tMax, HitRecord & collision) const override
	{
		bool hit_any = false;
		for(size_t i = 0; i < mMeshes.size(); ++i)
		{
			if(mMeshes[i].hit(r, tMin, tMax, collision))
			{
				collision.material = mMaterials[i].get();
				tMax = collision.t;
				hit_any = true;
			}
		}
		return hit_any;
	}

	const std::vector<TriangleMesh> mMeshes;
	const std::vector<std::shared_ptr<Material>> mMaterials;
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
	res.uv = math::lerp(uv,b.uv, x);
	return res;
}

//-------------------------------------------------------------------------------------------------
template<typename Idx>
TriangleMesh::TriangleMesh(
	const std::vector<VtxInfo>& vertices,
	const std::vector<Idx>& indices)
{
	mVtxData = vertices;
	mIndices.resize(indices.size());

	for(auto i = 0; i < mIndices.size(); ++i)
	{
		mIndices[i] = uint16_t(indices[i]);
	}
}

//-------------------------------------------------------------------------------------------------
inline bool TriangleMesh::hit(const math::Ray & r, float tMin, float tMax, HitRecord & collision) const
{
	auto nTris = mIndices.size() / 3;
	bool hit_any = false;
	for(int i = 0; i < nTris; ++i)
	{
		auto i0 = mIndices[3*i+0];
		auto i1 = mIndices[3*i+1];
		auto i2 = mIndices[3*i+2];
		auto tri = Triangle(mVtxData[i0].position, mVtxData[i1].position, mVtxData[i2].position);
		float f0, f1;
		if(tri.hit(r, tMin, tMax, collision, f0, f1))
		{
			hit_any = true;
			tMax= collision.t;
		}
	}

	return hit_any;
}