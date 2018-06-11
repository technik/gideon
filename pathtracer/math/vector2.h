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

	class Vec2f
	{
	public:
		Vec2f() = default;
		constexpr Vec2f(std::array<float,2> _list) : m(_list) {}
		Vec2f(std::initializer_list<float> il)
		{
			auto iter = il.begin();
			for(size_t i = 0; i < 2; ++i)
				m[i] = *iter++;
		}
		constexpr Vec2f(float k) : m({k,k}) {}
		constexpr Vec2f(float x, float y) : m({x,y}) {}

		// Vector accessors
		float x() const { return m[0]; }
		float y() const { return m[1]; }
		float& x() { return m[0]; }
		float& y() { return m[1]; }

		// Indexed accessor
		float operator[](size_t i) const { return m[i]; }
		float& operator[](size_t i) { return m[i]; }

		// Basic properties
		float norm() const { return std::sqrt(sqNorm()); }
		float sqNorm() const;
		void normalize() { *this*= (1.f/norm()); }

		// Math operators
		Vec2f operator-() const { return {-x(), -y()}; }
		Vec2f& operator+=(const Vec2f& v) {
			for(size_t i = 0; i < 2; ++i)
				m[i] += v.m[i];
			return *this;
		}

		Vec2f& operator-=(const Vec2f& v) {
			for(size_t i = 0; i < 2; ++i)
				m[i] -= v.m[i];
			return *this;
		}

		Vec2f& operator*=(const Vec2f& v) {
			for(size_t i = 0; i < 2; ++i)
				m[i] *= v.m[i];
			return *this;
		}

		Vec2f& operator/=(const Vec2f& v) {
			for(size_t i = 0; i < 2; ++i)
				m[i] /= v.m[i];
			return *this;
		}

		Vec2f& operator*=(float x) {
			for(size_t i = 0; i < 2; ++i)
				m[i] *= x;
			return *this;
		}

		Vec2f& operator/=(float x) {
			for(size_t i = 0; i < 2; ++i)
				m[i] /= x;
			return *this;
		}

	private:
		std::array<float,2> m;
	};

	// External operators
	inline Vec2f operator+(const Vec2f& a, const Vec2f& b)
	{
		return {a.x()+b.x(), a.y()+b.y() };
	}

	inline Vec2f operator-(const Vec2f& a, const Vec2f& b)
	{
		return {a.x()-b.x(), a.y()-b.y() };
	}

	inline Vec2f operator*(const Vec2f& a, const Vec2f& b)
	{
		return {a.x()*b.x(), a.y()*b.y() };
	}

	inline Vec2f operator/(const Vec2f& a, const Vec2f& b)
	{
		return {a.x()/b.x(), a.y()/b.y() };
	}

	inline Vec2f operator+(const Vec2f& a, float b)
	{
		return {a.x()+b, a.y()+b };
	}

	inline Vec2f operator-(const Vec2f& a, float b)
	{
		return {a.x()-b, a.y()-b };
	}

	inline Vec2f operator*(const Vec2f& a, float b)
	{
		return {a.x()*b, a.y()*b };
	}

	inline Vec2f operator/(const Vec2f& a, float b)
	{
		return {a.x()/b, a.y()/b  };
	}

	inline Vec2f operator*(float b, const Vec2f& a)
	{
		return {a.x()*b, a.y()*b };
	}

	inline Vec2f operator/(float b, const Vec2f& a)
	{
		return {a.x()/b, a.y()/b };
	}

	inline bool operator==(const Vec2f& a, const Vec2f& b)
	{
		return a.x()==b.x()
			&& a.y()==b.y();
	}

	// Inline methods
	inline float dot(const Vec2f& a, const Vec2f& b)
	{
		return a.x()*b.x() + a.y()*b.y();
	}

	inline Vec2f reflect(const Vec2f& v, const Vec2f& n)
	{
		return v-2*dot(v,n)*n;
	}

	inline Vec2f min(const Vec2f& a, const Vec2f& b)
	{
		return {
			std::min(a.x(), b.x()),
			std::min(a.y(), b.y())
		};
	}

	inline Vec2f max(const Vec2f& a, const Vec2f& b)
	{
		return {
			std::max(a.x(), b.x()),
			std::max(a.y(), b.y())
		};
	}

	inline float Vec2f::sqNorm() const { return dot(*this, *this); }

	inline Vec2f normalize(const Vec2f& v)
	{
		return v * (1/v.norm());
	}
}
