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

namespace math {

	/// Base matrix view
	template<class Derived, class T, size_t m, size_t n>
	struct MatrixBaseView : BaseMatrixExpr<Derived,m,n>
	{
		// Assignment
		template<class OtherDerived>
		auto& operator=(const BaseMatrixExpr<OtherDerived,m,n>& v)
		{
			for(auto i = 0; i < n; ++i)
				(*this)[i] = v[i];
			return *this;
		}

		// Component access
		auto& operator[](size_t i) { return static_cast<Derived&>(*this)[i]; }

		// Other operations
		template<class Other>
		void operator+=(const BaseMatrixExpr<Other,m,n>& b)
		{ *this = static_cast<const Derived&>(*this)+b; }

		template<class Other>
		void operator-=(const BaseMatrixExpr<Other,m,n>& b)
		{ *this = static_cast<const Derived&>(*this)-b; }

		template<class Other>
		void operator*=(const BaseMatrixExpr<Other,m,n>& b)
		{ *this = static_cast<const Derived&>(*this)*b; }

		template<class Other>
		void operator/=(const BaseMatrixExpr<Other,m,n>& b)
		{ *this = static_cast<const Derived&>(*this)/b; }
	};

} // namespace math
