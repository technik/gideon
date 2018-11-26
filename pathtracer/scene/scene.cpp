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
#include "cmdLineParams.h"
#include "loadGltf.h"
#include <materials/splitSumMaterials.h>
#include <math/vector.h>
#include <memory>
#include <camera/sphericalCamera.h>
#include <camera/frustumCamera.h>
#include "scene.h"
#include <shapes/sphere.h>

using namespace std;
using namespace math;

//--------------------------------------------------------------------------------------------------
bool Scene::hit(
	const math::Ray& r,
	float tMax,
	HitRecord& collision
) const
{
	float t = tMax;
	bool hit_anything = false;
	for(auto h : mRenderables)
	{
		HitRecord tmp_hit;
		if(h->hit(r, t, tmp_hit))
		{
			collision = tmp_hit;
			t = tmp_hit.t;
			hit_anything = true;
		}
	}

	return hit_anything;
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
	if (params.testBallScene)
	{
		// Furnace environment
		const float furnaceLevel = 0.5f;
		auto probe = new FurnaceProbe(furnaceLevel);
		background = new GradientBackground(Vec3f(furnaceLevel), Vec3f(furnaceLevel));

		// Split sum material
		// Material* sphereMaterial = new SplitSumReflectorMS(probe, 1.f); // Perfect reflector
		Material* sphereMaterial = new CopperSS(probe, 1.0f); // Copper

		// Test sphere
		auto testBallShape = std::make_shared<Sphere>(Vec3f(0.f), 1.0f, sphereMaterial);
		Matrix34f xform = Matrix34f::identity();
		mRenderables.push_back(std::make_shared<MeshInstance>(testBallShape, xform));
	}
	else if(!params.scene.empty())
	{
		loadGltf(params.scene.c_str(), *this, float(params.sx)/params.sy, params.overrideMaterials);
	
		// Background
		if(params.background.empty())
		{
			background = new GradientBackground ({0.5f, 0.7f, 1.f}, Vec3f(1.f));
		}
		else
		{
			background = new HDRBackground(params.background.c_str());
		}
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