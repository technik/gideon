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
    return mTlas.hitClosest(r, tMax, collision);
}

//--------------------------------------------------------------------------------------------------
void Scene::generateRandomScene(RandomGenerator& random)
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

void Scene::buildTLAS()
{
    auto t0 = chrono::high_resolution_clock::now();
    auto dt = chrono::high_resolution_clock::now() - t0;

    mTlas.build(mRenderables);

    auto us = chrono::duration_cast<chrono::nanoseconds>(dt).count() * 0.001;

    if(us < 1)
        std::cout << "BVH construction: " << us << " micros\n";
    else
        std::cout << "BVH construction: " << us*0.001 << " ms\n";
}