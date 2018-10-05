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

#include <cstddef>
#include <iostream>
#include <vector>

#include <background.h>
#include "camera/frustumCamera.h"
#include "camera/sphericalCamera.h"
#include "cmdLineParams.h"
#include "math/rectangle.h"
#include "collision.h"
#include "scene/scene.h"
#include "scene/loadGltf.h"
#include "textures/image.h"
#include "threadPool.h"

// ------ Single header libraries ------
#define STBI_MSC_SECURE_CRT
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

using namespace math;
using namespace std;

namespace {
	constexpr int MAX_DEPTH = 10;
}

//--------------------------------------------------------------------------------------------------
Vec3f color(const Ray& r, const Scene& world, int& depth, RandomGenerator& random)
{
	assert(abs(r.direction().sqNorm()-1) < 1e-4f);
	constexpr float farPlane = 1e3f;
	HitRecord hit;
	if(world.hit(r, farPlane, hit))
	{
		Ray scattered;
		Vec3f attenuation;
		Vec3f emitted;
		if(depth < MAX_DEPTH && hit.material->scatter(r, hit, attenuation, emitted, scattered, random))
		{
			return color(scattered, world, ++depth, random) * attenuation + emitted;
		}
		return Vec3f(0.f);
	}
	else
	{
		return world.background->sample(r.direction());
	}
}

using Rect = math::Rectangle<size_t>;

struct TileMetrics
{
	int maxRecursion = 0;
	int totalRecursion = 0;
};

//--------------------------------------------------------------------------------------------------
void generateSampleUVsInWindow(
	RandomGenerator& random,
	Rect window,
	unsigned nSamples,
	Image& dst,
	std::vector<math::Vec2f>& uvs)
{
	const auto totalNx = dst.width();
	const auto totalNy = dst.height();

	uvs.reserve(window.area() * nSamples);
	float randomCarry = random.scalar();
	for (size_t i = window.y0; i < window.y1; ++i)
		for (size_t j = window.x0; j < window.x1; ++j)
			for (size_t s = 0; s < nSamples; ++s)
			{
				float u = float(j + randomCarry) / totalNx;
				randomCarry = random.scalar();
				float v = 1.f - float(i + randomCarry) / totalNy;
				uvs.emplace_back(u, v);
			}
}

//--------------------------------------------------------------------------------------------------
void traceImageSegment(
	const Scene& world,
	Rect window,
	Image& dst,
	RandomGenerator& random,
	unsigned nSamples,
	TileMetrics& metrics)
{
	std::vector<Vec2f> uvs;
	generateSampleUVsInWindow(random, window, nSamples, dst, uvs);

	auto& cam = *world.cameras().front();
	std::vector<Ray> rays(uvs.size());
	cam.get_rays(rays.size(), uvs.data(), rays.data());

	size_t k = 0;
	for(size_t i = window.y0; i < window.y1; ++i)
		for(size_t j = window.x0; j < window.x1; ++j)
		{
			Vec3f accum(0.f);
			for(size_t s = 0; s < nSamples; ++s)
			{
				int depth = 0;
				accum += color(rays[k++], world, depth, random);

				// Save metrics
				metrics.maxRecursion = std::max(depth, metrics.maxRecursion);
				metrics.totalRecursion += depth;
			}
			accum /= float(nSamples);

			dst.pixel(j,i) = accum;
		}
}

struct ThreadInfo
{
	RandomGenerator random;
	int index;
};

//--------------------------------------------------------------------------------------------------
int main(int _argc, const char** _argv)
{
	CmdLineParams params(_argc, _argv);
	Rect size {0, 0, params.sx, params.sy };

	Image outputImage(params.sx, params.sy);

	// Scene
	Scene world;
	world.loadFromCommandLine(params);

	// Divide the image in tiles that can be consumed as jobs
	if(!(size.x1%params.tileSize == 0) ||
		!(size.y1%params.tileSize == 0))
	{
		std::cout << "Incompatible tile and image size. Image size (" << size.x1 << "x" << size.y1 << ") must be an exact multiple of tile size (" << params.tileSize << ")\n";
		return -1;
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

	// Prepare independent data for each thread
	std::vector<ThreadInfo> threadData(params.nThreads);
	for(int i = 0; i < threadData.size(); ++i)
	{
		threadData[i].index = i;
	}

	// Debug images
	Image threadMap(xTiles, yTiles);
	Image depthMap(xTiles, yTiles);
	Image timeMap(xTiles, yTiles);

	// Allocate threads to consume
	ThreadPool taskQueue(params.nThreads);
	if(taskQueue.run(
		threadData,
		tiles,
		[&world,
		&outputImage,
		&timeMap,
		params]
		(ThreadInfo& ti, Rect& window){
			auto tileStart = std::chrono::high_resolution_clock::now();

			TileMetrics metrics;

			traceImageSegment(world, window, outputImage, ti.random, params.ns, metrics);
			auto x = window.x0 / params.tileSize;
			auto y = window.y0 / params.tileSize;

			std::chrono::duration<float> tileDuration = std::chrono::high_resolution_clock::now() - tileStart;

			timeMap.pixel(x,y) = 10.f*tileDuration.count();
		},
		cout))
	{
		// Save final image
		outputImage.saveAsSRGB(params.output.c_str());

		// Save debug images
		threadMap.saveAsLinearRGB("threadMap.png");
		depthMap.saveAsLinearRGB("depthMap.png");
		timeMap.saveAsLinearRGB("timeMap.png");
	};

	return -1; // Something failed, we shouldn't reach this point
}