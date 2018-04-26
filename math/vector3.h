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

#include <array>
#include <cmath>
#include <initializer_list>

namespace math {

	class Vec3f
	{
	public:
		Vec3f() = default;
		Vec3f(std::array<float,3> _list) : m(_list) {}
		Vec3f(float x, float y, float z) : m({x,y,z}) {}

		// Vector accessors
		float x() const { return m[0]; }
		float y() const { return m[1]; }
		float z() const { return m[2]; }
		float& x() { return m[0]; }
		float& y() { return m[1]; }
		float& z() { return m[2]; }

		// Color accessors
		float r() const { return m[0]; }
		float g() const { return m[1]; }
		float b() const { return m[2]; }
		float& r() { return m[0]; }
		float& g() { return m[1]; }
		float& b() { return m[2]; }

		// Indexed accessor
		float operator[](size_t i) const { return m[i]; }
		float& operator[](size_t i) { return m[i]; }

		// Basic properties
		float norm() const { return std::sqrt(sqNorm()); }
		float sqNorm() const;

		// Math operators
		Vec3f operator-() const { return {-x(), -y(), -z()}; }
		Vec3f& operator+=(const Vec3f& v) {
			for(size_t i = 0; i < 3; ++i)
				m[i] += v.m[i];
			return *this;
		}

		Vec3f& operator-=(const Vec3f& v) {
			for(size_t i = 0; i < 3; ++i)
				m[i] -= v.m[i];
			return *this;
		}

		Vec3f& operator*=(const Vec3f& v) {
			for(size_t i = 0; i < 3; ++i)
				m[i] *= v.m[i];
			return *this;
		}

		Vec3f& operator/=(const Vec3f& v) {
			for(size_t i = 0; i < 3; ++i)
				m[i] /= v.m[i];
			return *this;
		}

		Vec3f& operator*=(float x) {
			for(size_t i = 0; i < 3; ++i)
				m[i] *= x;
			return *this;
		}

		Vec3f& operator/=(float x) {
			for(size_t i = 0; i < 3; ++i)
				m[i] /= x;
			return *this;
		}

	private:
		std::array<float,3> m;
	};

	float dot(const Vec3f& a, const Vec3f& b)
	{
		return a.x()*b.x() + a.y()*b.y() + a.z()*b.z();
	}

	Vec3f cross(const Vec3f& a, const Vec3f& b)
	{
		return {
			a.y()*b.z()-a.z()*b.y(),
			a.z()*b.x()-a.x()*b.z(),
			a.x()*b.y()-a.y()*b.x()
		};
	}

	// External operators
	inline Vec3f operator+(const Vec3f& a, const Vec3f& b)
	{
		return {a.x()+b.x(), a.y()+b.y(), a.z()+b.z() };
	}

	inline Vec3f operator-(const Vec3f& a, const Vec3f& b)
	{
		return {a.x()-b.x(), a.y()-b.y(), a.z()-b.z() };
	}

	inline Vec3f operator*(const Vec3f& a, const Vec3f& b)
	{
		return {a.x()*b.x(), a.y()*b.y(), a.z()*b.z() };
	}

	inline Vec3f operator/(const Vec3f& a, const Vec3f& b)
	{
		return {a.x()/b.x(), a.y()/b.y(), a.z()/b.z() };
	}

	// Inline methods
	inline float Vec3f::sqNorm() const { return dot(*this, *this); }

}
