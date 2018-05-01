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


#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <iostream>
#include <random>
#include <vector>
#include <thread>

#include <background.h>
#include "math/ray.h"
#include "math/vector3.h"
#include "camera.h"

// ------ Single header libraries ------
#define STBI_MSC_SECURE_CRT
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

using namespace math;
using namespace std;

class RandomGenerator
{
public:
	float scalar()
	{
		return distrib(engine);
	}
	Vec3f unit_vector()
	{
		Vec3f p;
		do
		{
			p = 2*Vec3f(scalar(),scalar(),scalar())-1.f;
		} while(p.sqNorm() >= 1.f);
		return p;
	}
private:
	std::default_random_engine engine;
	std::uniform_real_distribution<float> distrib;
};

uint8_t floatToByteColor(float value)
{
	return uint8_t(std::max(0.f,std::min(value,1.f))*255); // Clamp value to the range 0-1, and convert to byte
}

//--------------------------------------------------------------------------------------------------
void saveImage(size_t width, size_t height, const std::vector<Vec3f>& img, const char* fileName)
{
	std::vector<uint8_t> tmpBuffer;
	tmpBuffer.reserve(img.size());
	for(auto c : img) {
		tmpBuffer.push_back(floatToByteColor(c.r()));
		tmpBuffer.push_back(floatToByteColor(c.g()));
		tmpBuffer.push_back(floatToByteColor(c.b()));
	}

	const int rowStride = 3*width;
	stbi_write_png(fileName, width, height, 3, tmpBuffer.data(), rowStride);
}

class Material;

struct HitRecord
{
	float t;
	Vec3f p;
	Vec3f normal;
	Material* material;
};

class Material
{
public:
	virtual bool scatter(const Ray& in, HitRecord& record, Vec3f& attenuation, Ray& out, RandomGenerator& random) const = 0;
};

class Lambertian : public Material
{
public:
	Lambertian(const Vec3f& c) : albedo(c) {}
	bool scatter(const Ray&, HitRecord& hit, Vec3f& attenuation, Ray& out, RandomGenerator& random) const override
	{
		auto target = hit.p + hit.normal + random.unit_vector();
		out = Ray(hit.p, target-hit.p);
		attenuation = albedo;
		return true;
	}

	Vec3f albedo;
};

class Metal : public Material
{
public:
	Metal(const Vec3f& c, float f) : albedo(c), fuzz(f) {}
	bool scatter(const Ray& in, HitRecord& hit, Vec3f& attenuation, Ray& out, RandomGenerator& random) const override
	{
		auto reflected = reflect(normalize(in.direction()), hit.normal);
		out = Ray(hit.p, reflected + random.unit_vector()*fuzz);
		attenuation = albedo;
		return dot(out.direction(), hit.normal) > 0.f;
	}

	Vec3f albedo;
	float fuzz;
};

class Sphere
{
public:
	Sphere(){}
	Sphere(const Vec3f& center, float radius, Material* mat)
		: mCenter(center)
		, mSqRadius(radius*radius)
		, m(mat)
	{}

	bool hit(
		const Ray& r,
		float tMin,
		float tMax,
		HitRecord& collision
	) const
	{
		auto ro = r.origin() - mCenter; // Ray origin relative to sphere's center
		float a = r.direction().sqNorm();
		float b = dot(ro, r.direction());
		float c = ro.sqNorm() - mSqRadius;
		auto discriminant = b*b-a*c;
		if(discriminant >= 0)
		{
			float t = (-b - sqrt(discriminant)) / a;
			if(t > tMin && t < tMax) {
				collision.t = t;
				collision.p = r.at(t);
				collision.normal = normalize(collision.p - mCenter);
				collision.material = m;
				return true;
			}
		}
		return false;
	}

private:
	Vec3f mCenter;
	float mSqRadius;
	Material* m;
};

//--------------------------------------------------------------------------------------------------
class World
{
public:
	World(){}
	World(std::vector<Sphere>&& lst) : mHitables(std::move(lst)) {}

	bool hit(const Ray& r, float tMin, float tMax, HitRecord& collision) const
	{
		float t = tMax;
		bool hit_anything = false;
		for(const auto& h : mHitables)
		{
			HitRecord tmp_hit;
			if(h.hit(r, tMin, t, tmp_hit))
			{
				collision = tmp_hit;
				t = tmp_hit.t;
				hit_anything = true;
			}
		}

		return hit_anything;
	}

private:
	std::vector<Sphere> mHitables;
};

GradientBackground skyBg({0.5f, 0.7f, 1.f}, Vec3f(1.f));

//--------------------------------------------------------------------------------------------------
Vec3f color(const Ray& r, const World& world, int depth, RandomGenerator& random)
{
	HitRecord hit;
	if(world.hit(r, 1e-5f, INFINITY, hit))
	{
		Ray scattered;
		Vec3f attenuation;
		if(depth < 10 && hit.material->scatter(r, hit, attenuation, scattered, random))
			return attenuation * color(scattered, world, depth+1, random);
		return Vec3f(0.f);
	}
	else
	{
		auto unitDirection = normalize(r.direction());
		return skyBg.sample(unitDirection); // Blend between white & sky color
	}
}

//--------------------------------------------------------------------------------------------------
constexpr size_t N_SAMPLES = 64u;

struct Rect
{
	Rect() = default;
	constexpr Rect(size_t _x0, size_t _y0, size_t _x1, size_t _y1)
		:x0(_x0), y0(_y0), x1(_x1), y1(_y1)
	{}
	size_t x0, y0, x1, y1;
	size_t nPixels() const { return (x1-x0)*(y1-y0); }
};

//--------------------------------------------------------------------------------------------------
void traceImageSegment(const Camera& cam, const World& world, Rect w, int totalNx, size_t totalNy, Vec3f* outputBuffer, RandomGenerator& random)
{
	for(size_t j = w.y0; j < w.y1; ++j)
		for(size_t i = w.x0; i < w.x1; ++i)
		{
			Vec3f accum(0.f);
			for(size_t s = 0; s < N_SAMPLES; ++s)
			{
				float u = float(i+random.scalar())/totalNx;
				float v = 1.f-float(j+random.scalar())/totalNy;
				Ray r = cam.get_ray(u,v);
				accum += color(r, world, 0, random);
			}
			accum /= float(N_SAMPLES);

			outputBuffer[i+totalNx*j] = Vec3f(
				std::pow(accum.x(), 1.f/2.23f),
				std::pow(accum.y(), 1.f/2.23f),
				std::pow(accum.z(), 1.f/2.23f));
		}
}

//--------------------------------------------------------------------------------------------------
void threadRoutine(
	const Camera& cam,
	const World& world,
	Rect imgSize,
	Vec3f* outputBuffer,
	const std::vector<Rect>& tiles,
	std::atomic<size_t>* tileCounter)
{
	RandomGenerator random;

	size_t selfCounter = (*tileCounter)++;
	while(selfCounter < tiles.size()) // Valid job
	{
		auto& tile = tiles[selfCounter];
		traceImageSegment(cam, world, tile, imgSize.x1, imgSize.y1, outputBuffer, random);
		selfCounter = (*tileCounter)++;
	}
}

//--------------------------------------------------------------------------------------------------
std::vector<Sphere> randomScene()
{
	RandomGenerator random;
	std::vector<Sphere> sList;
	sList.reserve(1+21*21);
	for(int i = 0; i < 21; ++i)
		for(int j = 0; j < 21; ++j)
		{
			Vec3f albedo = {random.scalar(), random.scalar(), random.scalar()};
			Material* mat = nullptr;
			float p = random.scalar();
			if(p > 0.2f)
				mat = new Lambertian(albedo);
			else
				mat = new Metal(albedo, random.scalar()*0.2f);
			float h = 0.2f;
			Vec3f center = Vec3f(float(i-10), h-0.5f, float(j-10)+-2.f) + Vec3f(0.8f*random.scalar(),0.f,0.8f*random.scalar());
			// Add the sphere
			sList.emplace_back(center, h, mat);
		}
	sList.emplace_back(Vec3f(0.f, -1000.5f, 0.f), 1000.f, new Lambertian(Vec3f(0.5f)));
	return sList;
}

//--------------------------------------------------------------------------------------------------
int main(int, const char**)
{
	constexpr Rect size {0, 0, 512, 256 };

	std::vector<Vec3f> outputBuffer(size.nPixels());
	auto world = World(randomScene());

	Vec3f camPos {0.f, 0.f, 0.f};
	Vec3f camLookAt { 0.f, 0.f, -1.f };
	Camera cam(camPos, camLookAt, 3.14159f*90/180, size.x1, size.y1);
	// Divide the image in tiles that can be consumed as jobs
	constexpr size_t yTiles = 8;
	constexpr size_t xTiles = 2*yTiles;
	static_assert(size.x1%xTiles == 0);
	static_assert(size.y1%yTiles == 0);
	std::vector<Rect> tiles;
	tiles.reserve(xTiles*yTiles);
	for(size_t j = 0; j < yTiles; ++j)
		for(size_t i = 0; i < xTiles; ++i)
		{
			size_t tileSx = size.x1/xTiles;
			size_t tileSy = size.y1/yTiles;
			tiles.emplace_back(i*tileSx, j*tileSy, tileSx*(i+1u), tileSy*(j+1u));
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
		ts[i] = std::thread(threadRoutine, cam, world, size, outputBuffer.data(), tiles, &tileCounter);
		if(!ts[i].joinable())
		{
			return -1;
		}
	}
	for(int i = 0; i < nThreads; ++i)
		ts[i].join();

	chrono::duration<double> runningTime = chrono::high_resolution_clock::now() - start;
	auto seconds = runningTime.count();
	auto numRays = size.x1*size.y1*N_SAMPLES;
	cout << "Running time: " << seconds << "\nRays per second: " << numRays/seconds << "\n";

	// Save final image
	saveImage(size.x1, size.y1, outputBuffer, "render.png");

	return 0;
}