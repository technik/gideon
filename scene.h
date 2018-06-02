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

#include <math/aabb.h>
#include <math/vector3.h>
#include <math/quaterrnion.h>
#include "random.h"
#include "shapes/sphere.h"
#include "shapes/triangleMesh.h"
#include "shapes/meshInstance.h"
#include <vector>
#include <fx/gltf.h>
#include <fstream>

namespace {
	//----------------------------------------------------------------------------------------------
	inline auto loadMaterials(
		const fx::gltf::Document& _document,
		const std::vector<std::shared_ptr<PBRMaterial::Sampler>>& _textures
	)
	{
		std::vector<std::shared_ptr<PBRMaterial>> materials;
		auto baseColor = math::Vec3f(1.f);

		// Load materials
		for(auto& matDesc : _document.materials)
		{
			std::shared_ptr<PBRMaterial::Sampler> albedo, physics, ao;

			auto& pbrDesc = matDesc.pbrMetallicRoughness;
			if(!pbrDesc.empty())
			{
				// Base color
				if(!pbrDesc.baseColorTexture.empty())
				{
					auto albedoNdx = pbrDesc.baseColorTexture.index;
					albedo = _textures[albedoNdx];
				}
				// Base color factor
				{
					auto& colorDesc = pbrDesc.baseColorFactor;
					baseColor = reinterpret_cast<const math::Vec3f&>(colorDesc);
				}
				// Metallic-roughness
				if(!pbrDesc.metallicRoughnessTexture.empty())
				{
					// Load map in linear space!!
					auto ndx = pbrDesc.metallicRoughnessTexture.index;
					physics = _textures[ndx];
				}
				/*if(pbrDesc.roughnessFactor != 1.f)
					mat->addParam("uRoughness", pbrDesc.roughnessFactor);
				if(pbrDesc.metallicFactor != 1.f)
					mat->addParam("uMetallic", pbrDesc.metallicFactor);*/

			}

			auto mat = std::make_shared<PBRMaterial>(baseColor, albedo, physics, ao);
			materials.push_back(mat);
		}

		return materials;
	}

	//----------------------------------------------------------------------------------------------
	// TODO: This method assumes the texture is sRGB.
	// Instead, textures should be loaded on demand, when real color space info is available, or a first pass
	// should be performed on materials, marking textures with their corresponding color spaces
	auto loadTextures(const std::string& _assetsFolder, const fx::gltf::Document& _document)
	{
		std::vector<std::shared_ptr<PBRMaterial::Sampler>> textures;
		textures.reserve(_document.textures.size());
		for(auto& textDesc : _document.textures)
		{
			// TODO: Use texture sampler information
			//auto& sampler = _document.samplers[textDesc.sampler];
			auto& image = _document.images[textDesc.source];
			textures.push_back(std::make_shared<PBRMaterial::Sampler>((_assetsFolder + image.uri).c_str()));
		}

		return textures;
	}
}

class Scene
{
public:
	Scene() {}
	Scene(const char* gltfFileName)
	{
		fx::gltf::Document document = fx::gltf::LoadFromText(gltfFileName);

		if(document.scene >= 0)
		{
			std::vector<math::Matrix34f> transforms(document.nodes.size());
			if(!loadTransforms(document, transforms))
				return;
			auto textures = loadTextures("", document);
			auto materials = loadMaterials(document, textures);
			loadMeshes(document, materials, document.meshes);
			for(int i = 0; i < document.nodes.size(); ++i)
			{
				const auto& node = document.nodes[i];
				if(node.mesh >= 0)
				{
					mShapes.push_back(new MeshInstance(*mMeshes[node.mesh], *mMaterials[node.mesh], transforms[i]));
				}
			}
		}
	}

	bool loadTransforms(const fx::gltf::Document& document, std::vector<math::Matrix34f>& transforms)
	{
		// Build index of parent nodes
		std::vector<int> parentIndices(document.nodes.size(), -1);
		// First, build hierarchy of nodes
		// Then, solve transforms
		for(int i = 0; i < document.nodes.size(); ++i)
		{
			auto& node = document.nodes[i];
			for(auto c : node.children)
			{
				parentIndices[c] = i;
			}
		}
		for(int i = 0; i < document.nodes.size(); ++i)
		{
			auto& node = document.nodes[i];
			auto xForm = readTransform(node);
			auto parent = parentIndices[i];
			if(parent >= 0)
			{
				xForm = transforms[parent] * xForm;
			}
			transforms[i] = xForm;
		}
		return true;
	}

	math::Matrix34f readTransform(const fx::gltf::Node& node)
	{
		math::Matrix34f xForm = node.matrix;
		math::Matrix34f composedXForm = math::Quatf(node.rotation).rotationMtx();
		// TODO: Support scales
		//math::Vec3f scale = node.scale;
		composedXForm.position() = node.translation;
		return xForm*composedXForm;
	}

	void loadMeshes(const fx::gltf::Document& document, const std::vector<std::shared_ptr<PBRMaterial>>& materials, std::vector<fx::gltf::Mesh>& meshes)
	{
		std::vector<uint8_t> bufferData;
		loadRawBuffer(document.buffers[0].uri.c_str(), bufferData);

		for(auto& meshDesc : meshes)
		{
			auto& primitive = meshDesc.primitives[0];
			auto& idxAccessor = document.accessors[primitive.indices];

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
			using Vtx = TriangleMesh::VtxInfo;
			std::vector<Vtx> vertices;
			{
				auto byteOffset = posAccessor.byteOffset;
				auto count = posAccessor.count;
				auto& bv = document.bufferViews[posAccessor.bufferView];
				auto stride = bv.byteStride ? bv.byteStride : 12; // Assume tightly packed vec3fs
				auto viewData = &bufferData[bv.byteOffset];
				vertices.resize(count);
				for(size_t i = 0; i < count; ++i)
				{
					vertices[i].position = reinterpret_cast<math::Vec3f&>(viewData[stride*i+byteOffset]);
				}
			}
			// uvs
			auto& uvAccessor = document.accessors[primitive.attributes["TEXCOORD_0"]];
			{
				auto byteOffset = uvAccessor.byteOffset;
				auto count = uvAccessor.count;
				auto& bv = document.bufferViews[uvAccessor.bufferView];
				auto stride = bv.byteStride ? bv.byteStride : 8; // Assume tightly packed vec2fs
				auto viewData = &bufferData[bv.byteOffset];
				vertices.resize(count);
				for(size_t i = 0; i < count; ++i)
				{
					vertices[i].u = reinterpret_cast<float&>(viewData[stride*i+byteOffset+0]);
					vertices[i].v = -reinterpret_cast<float&>(viewData[stride*i+byteOffset+4]);
				}
			}
			// normals
			auto& nrmAccessor = document.accessors[primitive.attributes["NORMAL"]];
			{
				auto byteOffset = nrmAccessor.byteOffset;
				auto count = nrmAccessor.count;
				auto& bv = document.bufferViews[nrmAccessor.bufferView];
				auto stride = bv.byteStride ? bv.byteStride : 12; // Assume tightly packed vec2fs
				auto viewData = &bufferData[bv.byteOffset];
				vertices.resize(count);
				for(size_t i = 0; i < count; ++i)
				{
					vertices[i].normal = reinterpret_cast<math::Vec3f&>(viewData[stride*i+byteOffset]);
				}
			}


			mMeshes.push_back(new TriangleMesh(vertices, indices));
			mMaterials.push_back(materials[primitive.material]);
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
	std::vector<TriangleMesh*>	mMeshes;
	std::vector<std::shared_ptr<Material>>	mMaterials;
};