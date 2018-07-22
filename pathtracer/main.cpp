//-------------------------------------------------------------------------------------------------
// Toy path tracer
//-------------------------------------------------------------------------------------------------
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

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <iostream>
#include <vector>
#include <thread>

#include <background.h>
#include "camera/frustumCamera.h"
#include "camera/sphericalCamera.h"
#include "math/rectangle.h"
#include "collision.h"
#include "scene/scene.h"
#include "scene/loadGltf.h"
#include "textures/image.h"

// ------ Single header libraries ------
#define STBI_MSC_SECURE_CRT
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

using namespace math;
using namespace std;

Background* skyBg = nullptr;

int factorial(int x)
{
	if(x <= 1)
		return 1;
	return x*factorial(x-1);
}

float shNorm(int l, int m)
{
	auto num = (2*l+1)*factorial(l-abs(m));
	auto den = 4*math::Constants<float>::pi*factorial(l+abs(m));
	return sqrt(num/den);
}

float legendre(int l, int m, float sinTheta, float z)
{
	assert(m <= l);
	if(m == 0 && l == 0)
	{
		return 1.f;
	}
	else
	{
		float zp = z*pow(sinTheta,m);
		if(m == l)
		{
			return (1-2*m)*legendre(l-1,m-1,sinTheta,z);
		}
		else if(l == m+1)
		{
			return (2*m+1)*zp*legendre(m,m,sinTheta, z);
		} else
		{
			auto num = (2*l-1)*zp*legendre(l-1,m,sinTheta, z) - (l+m-1)*legendre(l-2,m,sinTheta,z);
			auto den = l-m;
			return num/den;
		}
	}
}

//--------------------------------------------------------------------------------------------------
float sh(int l, int m, float cosTheta, float phi)
{
	float sinTheta = sqrt(1-cosTheta*cosTheta);
	if(m > 0)
	{
		return sqrt(2.f)*shNorm(l,m)*cos(m*phi)*legendre(l,m,sinTheta,cosTheta);
	}else if(m < 0)
	{
		return sqrt(2.f)*shNorm(l,m)*sin(abs(m)*phi)*legendre(l,abs(m),sinTheta,cosTheta);
	}
	else
		return shNorm(l,0)*legendre(l,0,sinTheta,cosTheta);
}

//--------------------------------------------------------------------------------------------------
Vec3f color(const Ray& r, const Scene& world, int depth, RandomGenerator& random)
{
	constexpr float farPlane = 1e3f;
	HitRecord hit;
	if(world.hit(r, farPlane, hit))
	{
		Ray scattered;
		Vec3f attenuation;
		if(depth < 10 && hit.material->scatter(r, hit, attenuation, scattered, random))
		{
			return attenuation * color(scattered, world, depth+1, random);
		}
		return Vec3f(0.f);
	}
	else
	{
		//return Vec3f(0.f);
		auto unitDirection = normalize(r.direction());
		return skyBg->sample(unitDirection);
	}
	// Spherical harmonics coefficients
	/*auto dir =r.direction();
	dir.normalize();
	auto phi = atan2(dir.y(),dir.x());
	auto cTheta = dir.z();
	auto shVal = sh(4,1,cTheta,phi);
	Vec3f c;
	c.r() = max(0.f,shVal);
	c.g() = abs(shVal)>1.001f?1.f:0.f;
	c.b() = max(0.f,-shVal);
	return c;*/
}

using Rect = math::Rectangle<size_t>;

//--------------------------------------------------------------------------------------------------
void traceImageSegment(
	const Scene& world,
	Rect window,
	Image& dst,
	RandomGenerator& random,
	unsigned nSamples)
{
	const auto totalNx = dst.width();
	const auto totalNy = dst.height();
	auto& cam = *world.cameras().front();

	for(size_t i = window.y0; i < window.y1; ++i)
		for(size_t j = window.x0; j < window.x1; ++j)
		{
			Vec3f accum(0.f);
			for(size_t s = 0; s < nSamples; ++s)
			{
				float u = float(j+random.scalar())/totalNx;
				float v = 1.f-float(i+random.scalar())/totalNy;
				Ray r = cam.get_ray(u,v);
				accum += color(r, world, 0, random);
			}
			accum /= float(nSamples);

			dst.pixel(j,i) = accum;
		}
}

//--------------------------------------------------------------------------------------------------
// Would prefer to pass things by reference, but std::thread creates local copies of the objects in that case
void threadRoutine(
	const Scene* world,
	Image* dst,
	const std::vector<Rect>* tiles,
	std::atomic<size_t>* tileCounter,
	unsigned nSamples)
{
	assert(world);
	assert(dst);
	assert(tiles);
	assert(tileCounter);

	RandomGenerator random;

	size_t selfCounter = (*tileCounter)++;
	while(selfCounter < tiles->size()) // Valid job
	{
		auto& tile = (*tiles)[selfCounter];
		traceImageSegment(*world, tile, *dst, random, nSamples);
		cout << selfCounter << "\n";
		selfCounter = (*tileCounter)++;
	}
}

struct CmdLineParams
{
	string scene;
	string background;
	string output = "render.png";
	unsigned sx = 640;
	unsigned sy = 480;
	unsigned ns = 4;
	bool overrideMaterials = false;
	float fov = 45.f;
	unsigned tileSize = 20;
	bool sphericalRender = false;

	int process(const vector<string>& args, int i)
	{
		auto& arg = args[i];
		if(arg == "-bg") {
			background = args[i+1];
			return 2;
		}
		if(arg == "-scene")
		{
			scene = args[i+1];
			return 2;
		}
		if(arg == "-solid")
		{
			overrideMaterials = true;
			return 1;
		}
		if(arg == "-o")
		{
			output = args[i+1];
			return 2;
		}
		if(arg == "-s") // samples per pixel
		{
			ns = atoi(args[i+1].c_str());
			return 2;
		}
		if(arg == "-w")
		{
			sx = atoi(args[i+1].c_str());
			return 2;
		}
		if(arg == "-h")
		{
			sy = atoi(args[i+1].c_str());
			return 2;
		}
		if(arg == "-fov")
		{
			fov = (float)atof(args[i+1].c_str());
			return 2;
		}
		if(arg == "-tile")
		{
			tileSize = atoi(args[i+1].c_str());
			return 2;
		}
		if(arg == "-fullHD")
		{
			sx = 1920;
			sy = 1080;
			return 1;
		}
		if(arg == "-spherical")
		{
			sphericalRender = true;
			return 1;
		}
		return 1;
	}

	CmdLineParams(int _argc, const char** _argv)
	{
		vector<string> args(_argc);
		// Read all params
		int i = 0;
		for(auto& s : args)
			s = _argv[i++];
		i = 0;
		while(i < _argc)
			i += process(args, i);
	}
};

//--------------------------------------------------------------------------------------------------
int main(int _argc, const char** _argv)
{
	CmdLineParams params(_argc, _argv);
	Rect size {0, 0, params.sx, params.sy };

	Image outputImage(params.sx, params.sy);

	// Scene
	Scene world;
	if(!params.scene.empty())
	{
		loadGltf(params.scene.c_str(), world, float(params.sx)/params.sy, params.overrideMaterials);
	}

	// Background
	if(params.background.empty())
	{
		skyBg = new GradientBackground ({0.5f, 0.7f, 1.f}, Vec3f(1.f));
	}
	else
	{
		skyBg = new HDRBackground(params.background.c_str());
	}

	// Camera
	if(params.sphericalRender)
	{
		world.cameras().emplace_back(make_shared<SphericalCamera>(Vec3f(0.f), Vec3f{0.f,0.f,1.f}, Vec3f{0.f,0.f,1.f}));
	}
	if(world.cameras().empty()) // Create a default camera
	{
		Vec3f camPos { -1.0f, 0.0f, 4.f}; // Damaged helmet
		//Vec3f camPos { 189.95187377929688f, 579.0979614257813f, -386.1866149902344f }; // Reciprocating saw
		Vec3f camLookAt { 0.f, 0.f, 0.f };
		//Vec3f camPos { 0.f, 0.0f, 0.f};
		//Vec3f camLookAt { 0.f, 0.f, -1.f };
		world.cameras().emplace_back(make_shared<FrustumCamera>(camPos, camLookAt, 3.14159f*params.fov/180, float(size.x1)/size.y1));
	}

	// Divide the image in tiles that can be consumed as jobs
	if(!(size.x1%params.tileSize == 0) ||
		!(size.y1%params.tileSize == 0))
	{
		std::cout << "Incompatible tile and image size. Image size (" << size.x1 << "x" << size.y1 << ") must be an exact multiple of tile size (" << params.tileSize << ")\n";
	}
    const auto xTiles = size.x1 / params.tileSize;
    const auto yTiles = size.y1 / params.tileSize;
	std::vector<Rect> tiles;
	tiles.reserve(xTiles*yTiles);
	for(size_t j = 0; j < yTiles; ++j)
		for(size_t i = 0; i < xTiles; ++i)
		{
			tiles.emplace_back(i*params.tileSize, j*params.tileSize, params.tileSize*(i+1u), params.tileSize*(j+1u));
		}

	// Allocate threads to consume
	std::atomic<size_t> tileCounter = 0; // Atomic counter for lock-free jobs
	const int nThreads = 16;
	std::vector<std::thread> ts(nThreads);

	// Run jobs
	cout << "Running " << nThreads << " threads for " << tiles.size() << " tiles\n";
	auto start = chrono::high_resolution_clock::now();

	for(int i = 0; i < nThreads; ++i)
	{
		ts[i] = std::thread(threadRoutine, &world, &outputImage, &tiles, &tileCounter, params.ns);
		if(!ts[i].joinable())
		{
			return -1;
		}
	}
	for(int i = 0; i < nThreads; ++i)
		ts[i].join();

	chrono::duration<double> runningTime = chrono::high_resolution_clock::now() - start;
	auto seconds = runningTime.count();
	auto numRays = size.x1*size.y1*params.ns;
	cout << "Running time: " << seconds << "\nRays per second: " << numRays/seconds << "\n";

	// Save final image
	outputImage.saveAsSRGB(params.output.c_str());

	return 0;
}