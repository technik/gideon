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
#include "vector.h"

namespace math
{
	//-----------------------------------------------------------------
	// Explicitly SIMD set of 8 Vec3fs
	class float4
	{
	public:
		float4() = default;

		explicit float4(const std::array<float,4>& x) {
			m = _mm_set_ps(x[3], x[2], x[1], x[0]);
		}

		explicit float4(float x, float y, float z, float w) {
			m = _mm_set_ps(w, z, y, x);
		}

		explicit float4(Vec3f v)
		{
			m = _mm_set_ps(v.z(), v.z(), v.y(), v.x());
		}

		float4(float x)
		{
			m = _mm_set_ps1(x);
		}

		explicit float4(__m128 x) : m(x) {}

		float4 operator+(const float4& b) const {
			return float4(_mm_add_ps(m, b.m));
		}

		void operator+=(const float4& b) {
			m = _mm_add_ps(m, b.m);
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

		float4 operator>=(const float4& b) const {
			return float4(_mm_cmpge_ps(m, b.m));
		}

		bool any() const
		{
			return _mm_movemask_ps(m) != 0;
		}

		bool none() const
		{
			return _mm_movemask_ps(m) == 0;
		}

		bool all() const
		{
			return _mm_movemask_ps(m) == -1;
		}

		template<uint8_t a, uint8_t b, uint8_t c, uint8_t d>
		float4 shuffle() const
		{
			constexpr int mask = (d<<6)|(c<<4)|(b<<2)|a;
			return float4(_mm_permute_ps(m,mask));
		}

		float hMin() const;
		float hMax() const;

		float x() const {
			return _mm_cvtss_f32(m);
		}

		float y() const {
			auto t = shuffle<1,1,1,1>();
			return _mm_cvtss_f32(t.m);
		}

		float z() const {
			auto t = shuffle<2,2,2,2>();
			return _mm_cvtss_f32(t.m);
		}

		float w() const {
			auto t = shuffle<3,3,3,3>();
			return _mm_cvtss_f32(t.m);
		}

		__m128 m;
	};

	inline auto min(float4 a, float4 b)
	{
		return float4(_mm_min_ps(a.m,b.m));
	}

	inline auto max(float4 a, float4 b)
	{
		return float4(_mm_max_ps(a.m,b.m));
	}

	inline float float4::hMin() const
	{
		float4 v = min(*this, shuffle<2,3,0,1>());
		v = min(v,v.shuffle<1,0,3,2>());
		return v.x();
	}

	inline float float4::hMax() const
	{
		float4 v = max(*this, shuffle<2,3,0,0>());
		v = max(v,v.shuffle<1,0,0,0>());
		return v.x();
	}


	//-----------------------------------------------------------------
	// A single vec3 implemented using simd packed 4 floats
	class VecSimd3f
	{
	public:
		VecSimd3f() = default;
		explicit VecSimd3f(Vec3f v)
			: m( _mm_set_ps(v.x(),v.y(),v.z(),v.z()) )
		{}

		explicit VecSimd3f(__m128 x) : m(x) {}

		VecSimd3f operator+(const VecSimd3f& b) const {
			return VecSimd3f(_mm_add_ps(m, b.m));
		}

		VecSimd3f operator-(const VecSimd3f& b) const {
			return VecSimd3f(_mm_sub_ps(m, b.m));
		}

		VecSimd3f operator*(const VecSimd3f& b) const {
			return VecSimd3f(_mm_mul_ps(m, b.m));
		}

		void operator=(VecSimd3f b) { m = b.m; }

		__m128 operator<=(VecSimd3f b) const {
			return _mm_cmple_ps(m,b.m);
		}

		bool operator==(VecSimd3f b) { return float4(_mm_cmpeq_ps(m, b.m)).all(); }

		__m128 m;
	};

	inline auto min(VecSimd3f a, VecSimd3f b)
	{
		return VecSimd3f(_mm_min_ps(a.m,b.m));
	}

	inline auto max(VecSimd3f a, VecSimd3f b)
	{
		return VecSimd3f(_mm_max_ps(a.m,b.m));
	}

	//-----------------------------------------------------------------
	// A pack of 4 vec3 implemented using simd packed 4 floats
	using Vec3f4 = Vector3<float4>; // simd4 vectors of 3 components

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
