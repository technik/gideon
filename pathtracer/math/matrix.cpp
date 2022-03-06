//-------------------------------------------------------------------------------------------------
// Toy path tracer
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
// Apply gauss elimination with partial pivoting
#include "matrix.h"

namespace math {
	void Matrix44f::factorizationLU(Matrix44f& L, Matrix44f& U, std::array<int, 4>& P) const
	{
		P = { 0, 1, 2, 3 };
		L = identity();
		U = *this;
		// For each column
		for (auto k = 0; k < 4; k++)
		{
			// Find best pivot
			auto maxA = std::abs(U.element(k, k));
			auto bestI = k;
			for (auto i = k + 1; i < 4; ++i) // Find best pivot in the column
			{
				auto absA = std::abs(U.element(i, k));
				if (absA > maxA)
				{
					maxA = absA;
					bestI = i;
				}
			}
			constexpr float tolerance = 1e-4f;
			assert(maxA > tolerance); // Ensure minimun conditioning of the matrices

			// permutate pivot row
			if (k != bestI)
			{
				auto tk = P[bestI];
				P[bestI] = P[k];
				P[k] = tk;
				for (auto j = k; j < 4; ++j) // Swap all elements to the right of the pivot, in both rows k and p[k]
				{
					auto a = U(k, j);
					U(k, j) = U(bestI, j);
					U(bestI, j) = a;
				}
				for (auto j = 0; j < k; ++j) // Swap all elements to the right of the pivot, in both rows k and p[k]
				{
					auto a = L(k, j);
					L(k, j) = L(bestI, j);
					L(bestI, j) = a;
				}
			}

			// Partially solve all rows below the pivot
			auto pivot = U(k, k);
			for (auto i = k + 1; i < 4; ++i)
			{
				auto L_ik = U(i, k) / pivot; // Multiplier coefficient for row i
				L(i, k) = L_ik;
				for (auto j = k + 1; j < 4; ++j)
				{
					U(i, j) = U(i, j) - L_ik * U(k, j);
				}
				U(i, k) = 0; // U is upper triangular, so all elements below the diagonal must be 0
			}
		}
	}
}	// namespace math