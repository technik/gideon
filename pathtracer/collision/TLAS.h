#pragma once

#include "CWBVH.h"
#include "shapes/triangle.h"

class BLAS
{
public:
    void build();

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

class TLAS
{
public:
    // Construction
    // TODO: Add different construction methods: Embree, Morton codes, Surface area heuristic
    void build();
    void clear();

    // Queries
    float closestHit(const math::Ray& ray, float tMax) const
    {
        // Check against global aabb
        auto implicitRay = ray.implicit();
        if (!m_boundingBox.intersect(implicitRay, tMax))
            return false;

        // Init traversal stack to the root
        CWBVH::TraversalState stack;
        stack.reset(implicitRay, tMax);

        collision.t = -1;
        uint32_t closestHitId;

        while (m_bvh.continueTraverse(stack, closestHitId))
        {
            // Closest hit logic
            HitRecord hitInfo;
            if ((*m_instances)[closestHitId]->hit(ray, stack.tMax, hitInfo))
            {
                collision = hitInfo;
                stack.tMax = hitInfo.t;
            }
        }

        return collision.t >= 0;
    }

    bool anyHit() const;

private:
    math::AABB m_boundingBox;
    CWBVH m_bvh;

    // Needs an array of BLASs
    std::shared_ptr <BLAS[]> m_BLASBuffer;
};