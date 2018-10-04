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

#include <camera/camera.h>
#include "shapes/meshInstance.h"
#include <collision/shapeArray.h>
#include <vector>

struct CmdLineParams;
class RandomGenerator;
class Background;

class Scene
{
public:
	Scene() {}

	void addRenderable(const MeshInstance& renderable) 
	{
		mMeshes.append(renderable);
	}

	void addCamera(const std::shared_ptr<Camera>& cam)
	{
		mCameras.emplace_back(cam);
	}

	const std::vector<std::shared_ptr<Camera>>& cameras() const { return mCameras; }
	std::vector<std::shared_ptr<Camera>>& cameras() { return mCameras; }

	bool hit(
		const math::Ray& r,
		float tMax,
		HitRecord& collision
	) const;

	void generateRandomScene(RandomGenerator& random);
	void loadFromCommandLine(const CmdLineParams&);

	Background* background = nullptr;

private:
	ShapeArray<MeshInstance> mMeshes;
	std::vector<std::shared_ptr<Camera>>	mCameras;
};

//--------------------------------------------------------------------------------------------------
inline bool Scene::hit(
	const math::Ray& r,
	float tMax,
	HitRecord& collision
) const
{
	return mMeshes.hit(r, tMax, collision);
}
