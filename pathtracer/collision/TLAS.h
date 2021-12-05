#pragma once

#include "CWBVH.h"

class BLAS
{
public:
    void build();
    // update()

    float closestHit();
    float anyHit();

private:
};

class TLAS
{
public:
    // Construction
    // TODO: Add different construction methods: Embree, Morton codes, Surface area heuristic
    void build();
    void clear();

    // Queries
    float closestHit() const;
    bool anyHit() const;

private:
    BLAS* blas;
    math::AABB m_boundingBox;
    CWBVH m_bvh;

    // Needs an array of BLASs
};