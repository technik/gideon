#pragma once

#include "CWBVH.h"
#include "shapes/triangle.h"
#include "math/matrix.h"

class BLAS
{
public:
    void build(const math::Vec3f* vertices, const uint32_t* indices, uint32_t numTris)
    {
        // Compute triangle BBoxes
        std::vector<math::AABB> aabbs(numTris);
        for (uint32_t i = 0; i < numTris; ++i)
        {
            auto& triBBox = aabbs[i];
            triBBox.clear();
            auto i0 = indices[3 * i + 0];
            auto i1 = indices[3 * i + 1];
            auto i2 = indices[3 * i + 2];

            triBBox.add(vertices[i0]);
            triBBox.add(vertices[i1]);
            triBBox.add(vertices[i2]);
        }

        m_bvh.build(aabbs);
    }

    auto aabb() const { return m_bvh.aabb(); }

    // This method will assume you already checked against the AABB, and won't repeat that test.
    bool closestHit(const math::Ray& ray, float tMax, uint32_t& closestHitId)
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
                closestHitId = triangleHitId;
            }
        }

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
    CWBVH m_bvh;
    std::vector<Triangle::Simd> m_triangles; // I.e. bvh leafs
};