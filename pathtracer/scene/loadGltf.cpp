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

#include "loadGltf.h"
#include "scene.h"
#include <camera/frustumCamera.h>
#include <materials/Lambertian.h>
#include <materials/pbrMaterial.h>
#include "math/matrix.h"
#include "math/quaterrnion.h"
#include "math/vector.h"
#include <fx/gltf.h>

using namespace math;
using namespace fx;
using namespace std;

namespace { // Auxiliary functions

	//--------------------------------------------------------------------------------------------------
	inline string getFolder(const string& path)
	{
		auto pos = path.find_last_of("\\/");
		if (pos != string::npos)
			return path.substr(0, pos+1);

		return "";
	}

	//--------------------------------------------------------------------------------------------------
	math::Matrix34f readTransform(const gltf::Node& node)
	{
		// Directly use the matrix when available
		auto xForm = Matrix34f();
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 4; ++j)
				xForm(i,j) = node.matrix[4 * j + i];

		// Default values
		auto rot = Quatf({0.f, 0.f, 0.f, 1.f});
		auto trans = Vec3f(0.f);
		auto scale = Vec3f(1.f);
		// Read from the node
		rot = *reinterpret_cast<const Quatf*>(&node.rotation);
		trans = *reinterpret_cast<const Vec3f*>(&node.translation);
		scale = *reinterpret_cast<const Vec3f*>(&node.scale);
		// Reconstruct matrix
		auto scaleMatrix = Matrix34f(0.f);
		scaleMatrix(0,0) = scale[0];
		scaleMatrix(1,1) = scale[1];
		scaleMatrix(2,2) = scale[2];
		auto rotatedScale = rot.rotationMtx() * scaleMatrix;
		rotatedScale.position() = trans;
		return xForm * rotatedScale; // Concatenate both constructions, one of them will be identity
	}

	//--------------------------------------------------------------------------------------------------
	bool loadTransforms(const fx::gltf::Document& document, std::vector<math::Matrix34f>& transforms)
	{
		// Build index of parent nodes
		std::vector<int> parentIndices(document.nodes.size(), -1);
		// First, build hierarchy of nodes
		for(int i = 0; i < document.nodes.size(); ++i)
		{
			auto& node = document.nodes[i];
			for(auto c : node.children)
			{
				parentIndices[c] = i;
			}
		}
		// Read local transforms
		std::vector<math::Matrix34f> localTransforms(document.nodes.size());
		for(int i = 0; i < document.nodes.size(); ++i)
		{
			auto& node = document.nodes[i];
			localTransforms[i] = readTransform(node);
		}
		// Then, iterate over parent transforms reconstruct world transform.
		for(int i = 0; i < document.nodes.size(); ++i)
		{
			auto xForm = localTransforms[i];
			auto parent = parentIndices[i];
			while(parent >= 0)
			{
				xForm = localTransforms[parent] * xForm;
				parent = parentIndices[parent];
			}
			transforms[i] = xForm;
		}
		return true;
	}

	//----------------------------------------------------------------------------------------------
	auto loadMaterials(
		const fx::gltf::Document& _document,
		const std::vector<std::shared_ptr<PBRMaterial::Sampler>>& _textures,
		bool overrideMaterials
	)
	{
		std::vector<std::shared_ptr<Material>> materials;
		auto baseColor = math::Vec3f(1.f);

		// Load materials
		for(auto& matDesc : _document.materials)
		{
			if(overrideMaterials)
			{
				auto mat = std::make_shared<Lambertian>(Vec3f(0.75f));
				materials.push_back(mat);
				continue;
			}

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

	//----------------------------------------------------------------------------------------------
	template<class T, class TRead = T>
	std::vector<T> readAttribute(const fx::gltf::Document& document, const std::vector<uint8_t>& bufferData, uint32_t accessorNdx)
	{
		static_assert(sizeof(T) >= sizeof(TRead), "T can not contain TRead. Indices would be truncated");
		auto& accessor = document.accessors[accessorNdx];
		auto byteOffset = accessor.byteOffset;
		auto count = accessor.count;
		auto& bv = document.bufferViews[accessor.bufferView];
		auto viewData = &bufferData[bv.byteOffset];

		std::vector<T> data(count);
		auto stride = std::max<uint32_t>(bv.byteStride, sizeof(TRead));
		for(size_t i = 0; i < count; ++i)
		{
			data[i] = reinterpret_cast<const TRead&>(viewData[stride*i+byteOffset]);
		}

		return data;
	}

	//----------------------------------------------------------------------------------------------
	std::vector<uint16_t> readIndices(const fx::gltf::Document& document, const std::vector<uint8_t>& bufferData, uint32_t accessorNdx)
	{
		auto& accessor = document.accessors[accessorNdx];
		if(accessor.componentType == fx::gltf::Accessor::ComponentType::UnsignedByte)
			return readAttribute<uint16_t,uint8_t>(document, bufferData, accessorNdx);
		else 
			return readAttribute<uint16_t>(document, bufferData, accessorNdx);
	}

	//----------------------------------------------------------------------------------------------
    // Returns the list of BLAS ids for the loaded mesh's primitives
    std::vector<uint32_t> loadSingleMesh(
		const fx::gltf::Document& document,
        Scene& dstScene,
		const std::vector<uint8_t>& bufferData,
		const fx::gltf::Mesh& meshDesc,
		const std::vector<std::shared_ptr<Material>>& materials)
	{
		std::vector<uint32_t> primitives;
		for(auto& primitiveDesc : meshDesc.primitives)
		{
			auto indices = readIndices(document, bufferData, primitiveDesc.indices);
			auto position = readAttribute<math::Vec3f>(document, bufferData, primitiveDesc.attributes.at("POSITION"));

            auto numTris = indices.size() / 3;
            primitives.push_back(dstScene.addBlas(position.data(), indices.data(), numTris));
		}

		return primitives;
	}

	//----------------------------------------------------------------------------------------------
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

    struct MultiMesh
    {
        std::vector<uint32_t> primitives;
    };

	//----------------------------------------------------------------------------------------------
    // Returns the list of BLAS created for the loaded meshes
	std::vector<MultiMesh> loadMeshes(
        const std::string& _assetsFolder, const fx::gltf::Document& document,
        Scene& dstScene,
        const std::vector<std::shared_ptr<Material>>& materials)
	{
		std::vector<uint8_t> bufferData;
		loadRawBuffer((_assetsFolder+document.buffers[0].uri).c_str(), bufferData);

		std::vector<MultiMesh> meshes;
		for(auto& meshDesc : document.meshes)
		{
            MultiMesh mesh;
            mesh.primitives = loadSingleMesh(document, dstScene, bufferData, meshDesc, materials);
			meshes.push_back(mesh);
		}

		return meshes;
	}
}

//--------------------------------------------------------------------------------------------------
bool loadGltf(const char* fileName, Scene& dstScene, float aspectRatio, bool overrideMaterials)
{
	fx::gltf::Document document = fx::gltf::LoadFromText(fileName);

	if(document.scene < 0)
		return false;

	// Load transforms for all nodes
	auto transforms = std::vector<math::Matrix34f>(document.nodes.size());
	if(!loadTransforms(document, transforms))
		return false;

	// Optionally load a camera
	if(!document.cameras.empty())
	{
		int nodeNdx = 0;
		for(auto& node : document.nodes)
		{
			if(node.camera == 0)
			{
				auto& camDesc = document.cameras[0];
				auto xForm = transforms[nodeNdx];
				auto pos = xForm.transformPos(math::Vec3f(0.f));
				auto lookDir = xForm.transformDir({0.f,0.f,-1.f});
				dstScene.addCamera(make_shared<FrustumCamera>(pos, pos+lookDir, camDesc.perspective.yfov, aspectRatio));
				break;
			}
			++nodeNdx;
		}
	}

	auto folder = getFolder(fileName);
	vector<shared_ptr<PBRMaterial::Sampler>> textures;
	if(!overrideMaterials)
	{
		textures = loadTextures(folder, document);
	}
	auto materials = loadMaterials(document, textures, overrideMaterials);
	auto meshes = loadMeshes(folder, document, dstScene, materials);
	for(int i = 0; i < document.nodes.size(); ++i)
	{
		const auto& node = document.nodes[i];
        auto& pose = transforms[i];

		if(node.mesh >= 0)
		{
            auto& mesh = meshes[node.mesh];
            for (auto& primitive : mesh.primitives)
            {
                dstScene.addInstance(primitive, pose);
            }
		}
	}

	return true;
}