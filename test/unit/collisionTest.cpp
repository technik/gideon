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
#include "../../pathtracer/collision/BLAS.h"

using namespace math;

int main()
{
    const size_t numTris = 4;
    std::vector<Vec3f> vertices(3 * numTris);
    std::vector<uint16_t> indices(3 * numTris);
    for (size_t i = 0; i < numTris; ++i)
    {

        vertices[3 * i + 0] = Vec3f(i, -1, -1);
        vertices[3 * i + 1] = Vec3f(i, 0, 1);
        vertices[3 * i + 2] = Vec3f(i, 1+i, 0);
        indices[3 * i + 0] = 3 * i + 0;
        indices[3 * i + 1] = 3 * i + 1;
        indices[3 * i + 2] = 3 * i + 2;
    }

	// Create a blas with n consecutive points in line
    BLAS blas(vertices.data(), indices.data(), numTris);

    Ray ray(Vec3f(-1.f, 0.f, 0.f), Vec3f(1.f, 0.f, 0.f));

    uint32_t hitId;
    float tHit = -1;
    float tMax = 10.f;
    Vec3f normal;

    // Intersect first tri
    blas.closestHit(ray, tMax, hitId, tHit, normal);

    assert(tHit == 1.f);
    assert(hitId == 0);

    // Intersect second tri from inside
    ray.origin() = Vec3f(0.5f, 0.f, 0.f);
    blas.closestHit(ray, tMax, hitId, tHit, normal);

    assert(tHit == 0.5f);
    assert(hitId == 1);

    // Intersect second tri from the side.
    // This should make sure we can hit any part of the hierarchy
    for (int i = 0; i < numTris; ++i)
    {
        ray.origin() = Vec3f(-1.f, 0.5f + i, 0.f);
        blas.closestHit(ray, tMax, hitId, tHit, normal);

        assert(tHit == 1.f + i);
        assert(hitId == i);
    }


	return 0;
}