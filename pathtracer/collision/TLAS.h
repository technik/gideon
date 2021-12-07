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
        for(uint32_t i = 0; i < numTris; ++i)
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

class TLAS
{
public:
    struct Instance
    {
        math::Matrix34f pose;
        uint32_t BlasIndex;
    };

    // Construction
    // TODO: Add different construction methods: Embree, Morton codes, Surface area heuristic
    void build(std::shared_ptr <BLAS[]> blasBuffer, std::shared_ptr<Instance[]> instances, uint32_t numInstances)
    {
        std::vector<math::AABB> aabbs(numInstances);
        for(uint32_t i = 0; i < numInstances; ++i)
        {
            auto& instance = instances[i];
            aabbs[i] = instance.pose * blasBuffer[instance.BlasIndex].aabb();;
        }

        m_BLASBuffer = std::move(blasBuffer);
        m_instances = std::move(instances);
    }

    // Queries
    bool closestHit(const math::Ray& ray, float tMax) const
    {
        // Check against global aabb
        auto implicitRay = ray.implicit();
        if (!m_bvh.aabb().intersect(implicitRay, tMax))
            return false;

        // Init traversal stack to the root
        CWBVH::TraversalState stack;
        stack.reset(implicitRay, tMax);

        uint32_t instanceHitId;
        while (m_bvh.continueTraverse(stack, instanceHitId))
        {
            const auto& instance = m_instances[instanceHitId];
            // Transform ray to the local space
            math::Ray localRay(
                instance.pose.transformPos(ray.origin()),
                instance.pose.transformDir(ray.direction()));

            // Closest hit logic
            uint32_t triHitId;
            if (m_BLASBuffer[instance.BlasIndex].closestHit(localRay, stack.tMax, triHitId))
            {
                // TODO: Compute intersection details
                assert(false && "Not implemented"); return false; // Return false for obvious break
                return true;
            }
        }

        return false;
    }

    bool anyHit() const;

private:
    CWBVH m_bvh;

    // Needs an array of BLASs
    std::shared_ptr<Instance[]> m_instances;
    std::shared_ptr<BLAS[]> m_BLASBuffer;
};