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
#pragma once

#include <functional>
#include <math/vector.h>
#include <memory>

//--------------------------------------------------------------------------------------------------
class Image
{
	using img_deleter = std::function<void(void*)>;
	using data_ptr = std::unique_ptr<math::Vec3f[], img_deleter>;

public:
	Image(const char* fileName);

	Image(size_t nx, size_t ny);

	auto area() const { return sx*sy; }
	auto width() const { return sx; }
	auto height() const { return sy; }

	// Coordinates starting in the upper left corner
	const math::Vec3f& pixel(size_t x, size_t y) const
	{
		return mData[x+sx*y];
	}

	math::Vec3f& pixel(size_t x, size_t y)
	{
		return const_cast<math::Vec3f&>(((const Image*)this)->pixel(x,y));
	}

	void saveAsSRGB(const char* fileName) const;

	void saveAsLinearRGB(const char* fileName) const;

private:
	static uint8_t floatToByteColor(float value);
	static uint8_t floatToLinearByteColor(float value);

	size_t sx, sy;
	data_ptr mData;
};