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

#include <background.h>

#include <chrono>
#include <iostream>
#include <memory>

#include <collision/CWBVH.h>
#include "cmdLineParams.h"
#include "loadGltf.h"
#include <math/vector.h>
#include <camera/sphericalCamera.h>
#include <camera/frustumCamera.h>
#include <collision/BLAS.h>
#include "scene.h"

using namespace std;
using namespace math;

//--------------------------------------------------------------------------------------------------
bool Scene::hit(
	const math::Ray& r,
	float tMax,
	HitRecord& collision
) const
{
    return mTlas.closestHit(r, tMax, collision);
}

//--------------------------------------------------------------------------------------------------
void Scene::loadFromCommandLine(const CmdLineParams& params)
{
	// Geometry
	if(!params.scene.empty())
	{
		loadGltf(params.scene.c_str(), *this, float(params.sx)/params.sy, params.overrideMaterials);
        buildTLAS();
	}

	// Background
	if(params.background.empty())
	{
		background = new GradientBackground ({0.5f, 0.7f, 1.f}, Vec3f(1.f));
	}
	else
	{
		background = new HDRBackground(params.background.c_str());
	}

	// Camera
	if(params.sphericalRender)
	{
		mCameras.emplace_back(make_shared<SphericalCamera>(Vec3f(0.f), Vec3f{0.f,0.f,1.f}, Vec3f{0.f,0.f,1.f}));
	}
	if(mCameras.empty()) // Create a default camera
	{
		Vec3f camPos { -1.0f, 0.0f, 4.f}; // Damaged helmet
										  //Vec3f camPos { 189.95187377929688f, 579.0979614257813f, -386.1866149902344f }; // Reciprocating saw
		Vec3f camLookAt { 0.f, 0.f, 0.f };
		//Vec3f camPos { 0.f, 0.0f, 0.f};
		//Vec3f camLookAt { 0.f, 0.f, -1.f };
		auto aspectRatio = float(params.sx)/params.sy;
		mCameras.emplace_back(make_shared<FrustumCamera>(camPos, camLookAt, 3.14159f*params.fov/180, aspectRatio));
	}
}

uint32_t Scene::addBlas(const math::Vec3f* vertices, const uint16_t* indices, uint32_t numTris)
{
    mBLASBuffer.push_back(BLAS(vertices, indices, numTris));
    return uint32_t(mBLASBuffer.size() - 1);
}

void Scene::buildTLAS()
{
    auto t0 = chrono::high_resolution_clock::now();

    mTlas.build(std::move(mBLASBuffer), std::move(mInstances));

    auto dt = chrono::high_resolution_clock::now() - t0;
    auto us = chrono::duration_cast<chrono::nanoseconds>(dt).count() * 0.001;

    if(us < 1)
        std::cout << "BVH construction: " << us << " micros\n";
    else
        std::cout << "BVH construction: " << us*0.001 << " ms\n";
}