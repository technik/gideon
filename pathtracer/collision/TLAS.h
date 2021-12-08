#pragma once

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
        std::shared_ptr <BLAS[]> blasBuffer,
        std::shared_ptr<Instance[]> instances,
        uint32_t numInstances);

    // Queries
    bool closestHit(const math::Ray& ray, float tMax) const;

    bool anyHit() const;

private:
    CWBVH m_bvh;

    // Needs an array of BLASs
    std::shared_ptr<Instance[]> m_instances;
    std::shared_ptr<BLAS[]> m_BLASBuffer;
};