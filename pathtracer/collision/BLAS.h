#pragma once

#include "CWBVH.h"
#include "shapes/triangle.h"
#include "math/matrix.h"

class BLAS
{
public:
    BLAS() = default;
    BLAS(const math::Vec3f* vertices, const uint16_t* indices, uint32_t numTris)
    {
        build(vertices, indices, numTris);
    }

    auto aabb() const { return m_bvh.aabb(); }

    // This method will assume you already checked against the AABB, and won't repeat that test.
    bool closestHit(const math::Ray& ray, float tMax, uint32_t& closestHitId, float& tOut, math::Vec3f& outNormal) const
    {
        // Init traversal stack to the root
        auto implicitRay = ray.implicit();
        auto simdRay = ray.simd();
        CWBVH::TraversalState stack;
        stack.reset(implicitRay, tMax);

        uint32_t triangleHitId = uint32_t(-1);

        while (m_bvh.continueTraverse(stack, triangleHitId))
        {
            // Closest hit logic
            float tHit = m_triangles[triangleHitId].hitNoBackface(simdRay);
            if (tHit >= 0.f && tHit <= stack.tMax)
            {
                stack.tMax = tHit;
                tOut = tHit;
                outNormal = m_triangles[triangleHitId].mNormal;
            }
        }

        closestHitId = triangleHitId;

        return triangleHitId != uint32_t(-1);
    }

    float anyHit(const math::Ray& ray, float tMax)
    {
        // Init traversal stack to the root
        auto implicitRay = ray.implicit();
        auto simdRay = ray.simd();
        CWBVH::TraversalState stack;
        stack.reset(implicitRay, tMax);

        uint32_t triangleHitId;

        while (m_bvh.continueTraverse(stack, triangleHitId))
        {
            // Closest hit logic
            float tHit = m_triangles[triangleHitId].hitNoBackface(simdRay);
            if (tHit >= 0.f && tHit <= stack.tMax)
            {
                return tHit;
            }
        }

        return -1;
    }

private:
    void build(const math::Vec3f* vertices, const uint16_t* indices, uint32_t numTris)
    {
        // Compute triangles and its bounding boxes
        m_triangles.reserve(numTris);
        std::vector<math::AABB> aabbs(numTris);

        for (uint32_t i = 0; i < numTris; ++i)
        {
            auto& triBBox = aabbs[i];
            triBBox.clear();
            auto i0 = indices[3 * i + 0];
            auto i1 = indices[3 * i + 1];
            auto i2 = indices[3 * i + 2];

            auto& v0 = vertices[i0];
            auto& v1 = vertices[i1];
            auto& v2 = vertices[i2];

            auto t = Triangle(v0, v1, v2);
            m_triangles.push_back(t.simd());
            triBBox.add(v0);
            triBBox.add(v1);
            triBBox.add(v2);
        }

        m_bvh.build(aabbs);
    }

    CWBVH m_bvh;
    std::vector<Triangle::Simd> m_triangles; // I.e. bvh leafs
};