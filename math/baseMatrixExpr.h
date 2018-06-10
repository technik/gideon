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

#include <algorithm>

namespace math {

	/// Base expression for all matrix-like types
	template<class Derived, size_t m, size_t n>
	struct BaseMatrixExpr
	{
		static constexpr size_t size = m*n;
		static constexpr size_t rows = m;
		static constexpr size_t cols = n;

		constexpr auto operator[](size_t i) const { return static_cast<const Derived&>(*this)[i]; }
	};

	/// Uniform matrix expression
	template<class T, size_t m, size_t n>
	struct UniformExpr : BaseMatrixExpr<UniformExpr<T,m,n>,m,n>
	{
		constexpr UniformExpr(T t) : x(t) {}
		constexpr auto operator[](size_t) const { return x; }

		const T x;
	};

	//-------------------------------------------------------------------------------------------------------
	// Generic operators
	//-------------------------------------------------------------------------------------------------------
	/// Generic unary operator
	template<class T, size_t m, size_t n, class Op>
	struct UnaryOp : BaseMatrixExpr<UnaryOp<T,m,n,Op>,m,n>
	{
		UnaryOp(
			const BaseMatrixExpr<T,m,n>& a,
			const Op& op) : mOp(op), mA(a)
		{}

		auto operator[](size_t i) const {
			return mOp(mA[i]);
		}

		const Op& mOp;
		const BaseMatrixExpr<T,m,n>& mA;
	};

	/// Binary opeartor on matrix expressions
	template<class T1, class T2, size_t m, size_t n, class Op>
	struct BinaryOp : BaseMatrixExpr<BinaryOp<T1,T2,m,n,Op>,m,n>
	{
		BinaryOp(
			const BaseMatrixExpr<T1,m,n>& a,
			const BaseMatrixExpr<T2,m,n>& b,
			const Op& op) : mOp(op), mA(a), mB(b)
		{}

		auto operator[](size_t i) const {
			return mOp(mA[i], mB[i]);
		}

		const Op& mOp;
		const BaseMatrixExpr<T1,m,n>& mA;
		const BaseMatrixExpr<T2,m,n>& mB;
	};

	//-------------------------------------------------------------------------------------------------------
	// Specialized operators
	//-------------------------------------------------------------------------------------------------------
	template<class T, size_t m, size_t n>
	auto operator-(const BaseMatrixExpr<T,m,n>& a)
	{
		return UnaryOp(a,[](auto a) { return -a; });
	}

	template<class T1, class T2, size_t m, size_t n>
	auto operator+(const BaseMatrixExpr<T1,m,n>& a, const BaseMatrixExpr<T2,m,n>& b)
	{
		return BinaryOp(a,b,[](auto a, auto b){ return a + b; });
	}

	template<class T1, class T2, size_t m, size_t n>
	auto operator-(const BaseMatrixExpr<T1,m,n>& a, const BaseMatrixExpr<T2,m,n>& b)
	{
		return BinaryOp(a,b,[](auto a, auto b){ return a - b; });
	}

	template<class T1, class T2, size_t m, size_t n>
	auto operator*(const BaseMatrixExpr<T1,m,n>& a, const BaseMatrixExpr<T2,m,n>& b)
	{
		return BinaryOp(a,b,[](auto a, auto b){ return a * b; });
	}

	template<class T1, class T2, size_t m, size_t n>
	auto operator/(const BaseMatrixExpr<T1,m,n>& a, const BaseMatrixExpr<T2,m,n>& b)
	{
		return BinaryOp(a,b,[](auto a, auto b){ return a / b; });
	}

	template<class T1, class T2, size_t m, size_t n>
	auto min(const BaseMatrixExpr<T1,m,n>& a, const BaseMatrixExpr<T2,m,n>& b)
	{
		return BinaryOp(a,b,[](auto a, auto b){ return std::min(a, b); });
	}

	template<class T1, class T2, size_t m, size_t n>
	auto max(const BaseMatrixExpr<T1,m,n>& a, const BaseMatrixExpr<T2,m,n>& b)
	{
		return BinaryOp(a,b,[](auto a, auto b){ return std::max(a, b); });
	}

} // namespace math
