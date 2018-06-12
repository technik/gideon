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
#include "../../pathtracer/math/matrix.h"
#include "../../pathtracer/math/quaterrnion.h"
#include "../../pathtracer/math/vector.h"
#include "../../pathtracer/math/random.h"
#include <cmath>
#include <vector>

using namespace math;

template<class Mat>
void testMatrixInverse(const Mat& m)
{
	auto mInv = m.inverse();
	auto reconstructedIdentity = mInv * m;
	for(int i = 0; i < 3; ++i)
	{
		for(int j = 0; j < 4; ++j)
		{
			auto error = 0.f;
			if(i == j) // Identity's 1s
			{
				error = abs(reconstructedIdentity(i,j)-1);
				assert(error < 5e-2f);
			}
			else { // Identity's 0s
				error = abs(reconstructedIdentity(i,j));
				assert(error < 5e-2f);
			}
		}
	}
}

int main()
{
	// Test characteristic matrices
	testMatrixInverse(Matrix44f::identity());
	{ // Scaled matrix
		Matrix44f m = Matrix44f::identity();
		for(int i = 0; i < 4; ++i)
			m(i,i) = 4.f;
		testMatrixInverse(m);
	}
	{ // Translated matrix
		Matrix44f m = Matrix44f::identity();
		for(int i = 0; i < 4; ++i)
			m(i,3) = 4.f;
		testMatrixInverse(m);
	} // Rotation matrices
	{
		std::vector<float> angles = { 0.f, 0.1f, 1.57f, 3.f, 3.14159f, 5.f };
		std::vector<Vec3f> axes = {
			{ 1.f, 0.f, 0.f },
			{ 0.f, 1.f, 0.f },
			{ 0.f, 0.f, 1.f },
			normalize(Vec3f(1.f, 1.f, 1.f))
		};
		for(auto axis : axes)
		{
			for(auto angle : angles)
			{
				auto c = cos(angle);
				auto s = sin(angle);
				Quatf rot = { axis.x() * s, axis.y() * s, axis.z() * s, c };
				testMatrixInverse(rot.rotationMtx());
			}
		}
	}
	// Test random matrices
	RandomGenerator g;
	constexpr int n = 1000;
	for(int i = 0; i < n; ++i)
	{
		auto angle = g.scalar();
		auto c = cos(angle);
		auto s = sin(angle);
		auto axis = g.unit_vector();
		Quatf q = {axis.x()*s, axis.y()*s, axis.z()*s, c};
		auto scale = g.unit_vector() * g.scalar() * 10.f;
		auto scaleMtx = Matrix34f::identity();
		for(int i = 0; i < 3; ++i)
			scaleMtx(i,i) = scale[i];
		auto trans = g.unit_vector() * g.scalar() * 10.f;
		Matrix34f m = q.rotationMtx() * scaleMtx;
		m.position() = trans;
		testMatrixInverse(m);
	}
	return 0;
}