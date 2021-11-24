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
#include "materials/Lambertian.h"
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
    constexpr int MAX_BOUNCES = 9;
}

//--------------------------------------------------------------------------------------------------
Vec3f color(Ray r, const Scene& world, RandomGenerator& random)
{
	assert(abs(r.direction().sqNorm()-1) < 1e-4f); // Check ray direction

    int depth = 0;
	constexpr float farPlane = 1e3f;

    Vec3f accumLight = Vec3f(0.f);
    Vec3f accumAttenuation = Vec3f(1.f);
	HitRecord hit;

    while(depth <= MAX_BOUNCES)
    {
        if (world.hit(r, farPlane, hit))
        {
            // Evaluate light bounce
            Ray scatteredRay;
            Vec3f attenuation;
            Vec3f emitted;
            lambertScatter(r, hit.p, hit.normal, Vec3f(0.75), attenuation, emitted, scatteredRay, random);
            r = scatteredRay;

            // Integrate path
            accumLight += accumAttenuation * emitted;
            accumAttenuation *= attenuation;

            ++depth;
        }
        else
        {
            // Gather light from the background
            accumLight += accumAttenuation * world.background->sample(r.direction());
            break;
        }
    }

    return accumLight;
}

using Rect = math::Rectangle<size_t>;

//--------------------------------------------------------------------------------------------------
void renderTile(
	Rect window,
	const Scene& world,
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

				accum += color(r, world, random);
			}
			accum /= float(nSamples);

			dst.pixel(j,i) = accum;
		}
}

struct ThreadInfo
{
	RandomGenerator random;
};

//--------------------------------------------------------------------------------------------------
int main(int _argc, const char** _argv)
{
	CmdLineParams params(_argc, _argv);
	Rect size {0, 0, params.sx, params.sy };

	Image outputImage(params.sx, params.sy);

	// Scene
	Scene world;
    auto t0 = chrono::high_resolution_clock().now();
	world.loadFromCommandLine(params);
    auto loadTime = chrono::high_resolution_clock().now() - t0;
    cout << "Loaded acceleration structure in " << chrono::duration_cast<chrono::milliseconds>(loadTime).count() << " milliseconds\n";

	// Divide the image in tiles that can be consumed as jobs
	if(!(size.x1%params.tileSize == 0) ||
		!(size.y1%params.tileSize == 0))
	{
		std::cout << "Incompatible tile and image size. Image size (" << size.x1 << "x" << size.y1 << ") must be an exact multiple of tile size (" << params.tileSize << ")\n";
		return -1;
	}

	// Prepare independent data for each thread
	std::vector<ThreadInfo> threadData(params.nThreads);

	// Allocate threads to consume
	ThreadPool taskQueue(params.nThreads);

    // Dispatch compute
    const auto xTiles = (size.x1 + params.tileSize -1) / params.tileSize;
    const auto yTiles = (size.y1 + params.tileSize -1) / params.tileSize;

	if(taskQueue.dispatch(
        xTiles * yTiles,
		[xTiles, &threadData,
        &world,
		&outputImage, params]
		(size_t taskIndex, size_t workerIndex){
            // Compute the boundaries of the tile to be rendered by this thread
            Rect tile;
            auto tx = taskIndex % xTiles;
            tile.x0 = tx * params.tileSize;
            auto ty = taskIndex / xTiles;
            tile.y0 = ty * params.tileSize;
            tile.x1 = tile.x0 + params.tileSize;
            tile.y1 = tile.y0 + params.tileSize;

            renderTile(tile, world, outputImage, threadData[workerIndex].random, params.ns);
		},
		cout))
	{
		// Save final image
		outputImage.saveAsSRGB(params.output.c_str());

        return 0;
	};

	return -1; // Something failed, we shouldn't reach this point
}