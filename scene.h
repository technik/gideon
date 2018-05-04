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

#include <math/vector3.h>
#include <math/aabb.h>
#include "random.h"
#include "shapes/shape.h"
#include "shapes/sphere.h"
#include "shapes/triangleMesh.h"
#include <vector>
#include <fx/gltf.h>
#include <fstream>

class Scene
{
public:
	Scene() {}
	Scene(const char* gltfFileName)
	{
		fx::gltf::Document document = fx::gltf::LoadFromText(gltfFileName);

		if(document.scene >= 0)
		{
			auto& mesh = document.meshes[0];
			auto& primitive = mesh.primitives[0];
			auto& idxAccessor = document.accessors[primitive.indices];

			std::vector<uint8_t> bufferData;
			loadRawBuffer(document.buffers[0].uri.c_str(), bufferData);

			std::vector<size_t> indices;
			{
				auto byteOffset = idxAccessor.byteOffset;
				auto count = idxAccessor.count;
				auto& bv = document.bufferViews[idxAccessor.bufferView];
				auto viewData = &bufferData[bv.byteOffset];
				indices.resize(count);
				for(size_t i = 0; i < count; ++i)
				{
					indices[i] = reinterpret_cast<uint16_t&>(viewData[2*i+byteOffset]);
				}
			}

			auto& posAccessor = document.accessors[primitive.attributes["POSITION"]];
			auto bbox = math::AABB(
				reinterpret_cast<math::Vec3f&>(*posAccessor.min.data()),
				reinterpret_cast<math::Vec3f&>(*posAccessor.max.data()));
			std::vector<math::Vec3f> vertices;
			{
				auto byteOffset = posAccessor.byteOffset;
				auto count = posAccessor.count;
				auto& bv = document.bufferViews[posAccessor.bufferView];
				auto stride = bv.byteStride ? bv.byteStride : 12; // Assume tightly packed
				auto viewData = &bufferData[bv.byteOffset];
				vertices.resize(count);
				for(size_t i = 0; i < count; ++i)
				{
					vertices[i] = reinterpret_cast<math::Vec3f&>(viewData[stride*i+byteOffset]);
				}
			}

			mShapes.push_back(new TriangleMesh(vertices, indices, bbox));
		}
	}

	void loadRawBuffer(const char* uri, std::vector<uint8_t>& dst)
	{
		std::ifstream fs(uri, std::ios::binary+std::ios::ate);
		if(fs)
		{
			auto size = fs.tellg();
			fs.seekg(0, fs.beg);
			size -= fs.tellg();
			dst.resize(size);
			fs.read((char*)dst.data(), size);
		}
	}

	Scene(RandomGenerator& random)
	{
		mShapes.reserve(1+21*21);
		for(int i = 0; i < 21; ++i)
			for(int j = 0; j < 21; ++j)
			{
				math::Vec3f albedo = {random.scalar(), random.scalar(), random.scalar()};
				Material* mat = nullptr;
				float p = random.scalar();
				if(p > 0.2f)
					mat = new Lambertian(albedo);
				else
					mat = new Metal(albedo, random.scalar()*0.2f);
				float h = 0.2f;
				math::Vec3f center = math::Vec3f(float(i-10), h-0.5f, float(j-10)+-2.f) + math::Vec3f(0.8f*random.scalar(),0.f,0.8f*random.scalar());
				// Add the sphere
				mShapes.push_back(new Sphere(center, h, mat));
			}
		mShapes.emplace_back(new Sphere(math::Vec3f(0.f, -1000.5f, 0.f), 1000.f, new Lambertian(math::Vec3f(0.5f))));
	}

	//--------------------------------------------------------------------------------------------------
	bool hit(
		const math::Ray& r,
		float tMin,
		float tMax,
		HitRecord& collision
	) const
	{
		float t = tMax;
		bool hit_anything = false;
		for(auto h : mShapes)
		{
			HitRecord tmp_hit;
			if(h->hit(r, tMin, t, tmp_hit))
			{
				collision = tmp_hit;
				t = tmp_hit.t;
				hit_anything = true;
			}
		}

		return hit_anything;
	}

private:
	std::vector<Shape*>	mShapes;
};