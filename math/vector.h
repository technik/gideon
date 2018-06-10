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

#include "baseMatrixExpr.h"
#include "baseMatrixView.h"

namespace math {

	//-------------------------------------------------------------------------------------------------------
	// Base vector classes
	//-------------------------------------------------------------------------------------------------------
	template<class Derived, size_t n>
	using BaseVectorExpr = BaseMatrixExpr<Derived,n,1>;

	/// Vector Expression
	template<class Derived, size_t n>
	struct VectorExpression : BaseVectorExpr<Derived,n>
	{
		static_assert(n > 0);

		auto x() const { return (*this)[0]; }
		auto y() const { return (*this)[1]; static_assert(n>1, "Vector is too small to have a y component"); }
		auto z() const { return (*this)[2]; static_assert(n>2, "Vector is too small to have a z component"); }
		auto w() const { return (*this)[3]; static_assert(n>3, "Vector is too small to have a w component"); }
	};
	
	/// Vector View
	template<class Derived, class T, size_t n>
	struct VectorView : MatrixBaseView<Derived,T,n,1>
	{
		using MatrixBaseView<Derived,T,n,1>::MatrixBaseView;
		static_assert(n > 0);

		auto x() const { return (*this)[0]; }
		auto y() const { return (*this)[1]; static_assert(n>1, "Vector is too small to have a y component"); }
		auto z() const { return (*this)[2]; static_assert(n>2, "Vector is too small to have a z component"); }
		auto w() const { return (*this)[3]; static_assert(n>3, "Vector is too small to have a w component"); }

		auto& x() { return (*this)[0]; }
		auto& y() { return (*this)[1]; static_assert(n>1, "Vector is too small to have a y component"); }
		auto& z() { return (*this)[2]; static_assert(n>2, "Vector is too small to have a z component"); }
		auto& w() { return (*this)[3]; static_assert(n>3, "Vector is too small to have a w component"); }
	};

	//-------------------------------------------------------------------------------------------------------
	// Special vector classes
	//-------------------------------------------------------------------------------------------------------

	template<class T, int n>
	struct UnitVector : VectorExpression<UnitVector<T,n>,n>
	{
		UnitVector() = delete;
	};

	//-------------------------------------------------------------------------------------------------------
	// Generic vector classes
	//-------------------------------------------------------------------------------------------------------

	template<class T, int n>
	struct Vector
		: VectorView<Vector<T,n>,T,n>
	{
		T& operator[](size_t i) { return a[i]; }
		const T& operator[](size_t i) const { return a[i]; }

		Vector() = default;
		Vector(T _x)
			: Vector(UniformExpr<T,n,1>(_x))
		{}

		template<class Other>
		Vector(const BaseVectorExpr<Other,n>& v)
		{ 
			for(auto i = 0; i < n; ++i)
				a[i] = v[i];
		}

		Vector(T _x, T _y) : a{_x,_y} {}
		Vector(T _x, T _y, T _z) : a{_x,_y,_z} {}
		Vector(T _x, T _y, T _z, T _w) : a{_x,_y,_z,_w} {}

	private:
		T a[n];
	};

	//-------------------------------------------------------------------------------------------------------
	// Vector aliases
	//-------------------------------------------------------------------------------------------------------

	template<class T> using Vector2 = Vector<T,2>;
	template<class T> using Vector3 = Vector<T,3>;
	template<class T> using Vector4 = Vector<T,4>;
	using Vec2f = Vector2<float>;
	using Vec3f = Vector3<float>;
	using Vec4f = Vector4<float>;
	
} // namespace math
