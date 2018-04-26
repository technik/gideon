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

// ------ Single header libraries ------
#define STBI_MSC_SECURE_CRT
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include <algorithm>
#include <cstddef>
#include <vector>

#include "math/ray.h"
#include "math/vector3.h"

using namespace math;

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

//--------------------------------------------------------------------------------------------------
Vec3f color(const Ray& r)
{
	auto unitDirection = normalize(r.direction());
	float t = 0.5f + 0.5f * unitDirection.y();
	constexpr Vec3f SkyColor = {0.5f, 0.7f, 1.f};
	return Vec3f(1.f-t) + t*SkyColor; // Blend between white & sky color
}

//--------------------------------------------------------------------------------------------------
int main(int, const char**)
{
	constexpr size_t nx = 400u;
	constexpr size_t ny = 200u;

	std::vector<Vec3f> outputBuffer;
	outputBuffer.reserve(nx*ny*3);

	Vec3f ll_corner { -2.f, -1.f, -1.f };
	Vec3f horizontal {4.f, 0.f, 0.f };
	Vec3f vertical { 0.f, 2.f, 0.f };
	Vec3f origin(0.f);

	for(int j = ny-1; j >= 0; j--)
		for(int i = 0; i < nx; ++i)
		{
			float u = float(i)/nx;
			float v = float(j)/ny;
			Ray r = { origin, ll_corner + u*horizontal + v*vertical};
			outputBuffer.push_back(color(r));
		}

	saveImage(nx, ny, outputBuffer, "Wiii.png");

	return 0;
}