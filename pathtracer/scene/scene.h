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

#include <camera/frustumCamera.h>
#include <math/aabb.h>
#include <math/vector.h>
#include <math/quaterrnion.h>
#include "math/random.h"
#include "shapes/sphere.h"
#include "shapes/triangleMesh.h"
#include "shapes/meshInstance.h"
#include <vector>
#include <fx/gltf.h>
#include <fstream>

class Scene
{
public:
	Scene() {}

	void addRenderable(const std::shared_ptr<MeshInstance>& renderable) 
	{
		mRenderables.emplace_back(renderable);
	}

	void addCamera(const std::shared_ptr<Camera>& cam)
	{
		mCameras.emplace_back(cam);
	}

	const std::vector<std::shared_ptr<Camera>>& cameras() const { return mCameras; }
	std::vector<std::shared_ptr<Camera>>& cameras() { return mCameras; }

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
		for(auto h : mRenderables)
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

	//--------------------------------------------------------------------------------------------------
	void generateRandomScene(RandomGenerator& random)
	{
		// TODO: Recover this functionality
		/*mShapes.reserve(1+21*21);
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
		mShapes.emplace_back(new Sphere(math::Vec3f(0.f, -1000.5f, 0.f), 1000.f, new Lambertian(math::Vec3f(0.5f))));*/
	}

private:
	std::vector<std::shared_ptr<MeshInstance>>	mRenderables;
	std::vector<std::shared_ptr<Camera>>	mCameras;
};