#pragma once

#include "BLAS.h"
#include "CWBVH.h"
#include "shapes/triangle.h"
#include "math/matrix.h"

class BLAS;

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
    void build(
        std::vector<BLAS>&& blasBuffer,
        std::vector<Instance>&& instances);

    // Queries
    bool closestHit(const math::Ray& ray, float tMax, HitRecord& dst) const;

private:
    CWBVH m_bvh;

    // Needs an array of BLASs
    std::vector<Instance> m_instances;
    std::vector<math::Matrix34f> m_invInstancePoses;
    std::vector<BLAS> m_BLASBuffer;
};