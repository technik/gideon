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
#pragma once
// Vectorized float structs with syntax extended from that of regular floats

#include <immintrin.h>
#include <xmmintrin.h>
#include <zmmintrin.h>

#include <array>

namespace math
{
	//-----------------------------------------------------------------
	// Explicitly SIMD set of 8 Vec3fs
	class float4
	{
	public:
		float4() = default;

		explicit float4(const std::array<float,4>& x) {
			m = _mm_set_ps(x[0], x[1], x[2], x[3]);
		}

		explicit float4(float x)
		{
			m = _mm_set_ps1(x);
		}

		explicit float4(__m128 x) : m(x) {}

		float4 operator+(const float4& b) const {
			return float4(_mm_add_ps(m, b.m));
		}

		float4 operator-(const float4& b) const {
			return float4(_mm_sub_ps(m, b.m));
		}

		float4 operator*(const float4& b) const {
			return float4(_mm_mul_ps(m, b.m));
		}

		float4 operator/(const float4& b) const {
			return float4(_mm_div_ps(m, b.m));
		}

		float4 operator<=(const float4& b) const {
			return float4(_mm_cmple_ps(m, b.m));
		}

		bool any() const
		{
			return _mm_movemask_ps(m) != 0;
		}

	private:
		__m128 m;
	};

	//-----------------------------------------------------------------
	class VecSimd3f
	{};

	//-----------------------------------------------------------------
	// Explicitly SIMD set of 8 Vec3fs
	class float8
	{
	public:
		float8() = default;
		explicit float8(const float* p) {
			m = _mm256_load_ps(p);
		}

		explicit float8(const std::array<float,8>& p) {
			m = _mm256_set_ps(p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7]);
		}

		explicit float8(float x) {
			m = _mm256_set1_ps(x);
		}

		explicit float8(__m256 x) : m(x) {}

		float8 operator+(const float8& b) const
		{
			return float8(_mm256_add_ps(m, b.m));
		}

		float8 operator-(const float8& b) const
		{
			return float8(_mm256_sub_ps(m, b.m));
		}

		float8 operator*(const float8& b) const
		{
			return float8(_mm256_mul_ps(m, b.m));
		}

		float8 operator/(const float8& b) const
		{
			return float8(_mm256_div_ps(m, b.m));
		}

		// this*b + c;
		float8 mul_add(const float8& b, const float8& c)
		{
			return float8(_mm256_fmadd_ps(m,b.m,c.m));
		}

	private:
		__m256 m;
	};
}
