//-------------------------------------------------------------------------------------------------
// Toy path tracer
//-------------------------------------------------------------------------------------------------
// Copyright 2021 Carmelo J Fdez-Aguera
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
#pragma once

#include <memory>
#include <functional>
#include <vector>

#include <math/vector.h>
#include <math/aabb.h>

namespace math
{
    class Ray;
}

class MeshInstance;
struct HitRecord;

// Compressed wide BVH based on Karras 2017
class CWBVH
{
public:
    CWBVH();
    ~CWBVH();
    void build(std::vector<std::shared_ptr<MeshInstance>>& instances);

    //bool hitAny(const math::Ray&, float tMax);
    bool hitClosest(
        std::vector<uint32_t>& stack,
        const math::Ray&,
        float tMax,
        HitRecord& collision) const;

private:

    struct BranchNode
    {
        using BlasCallback = std::function<float(uint32_t leafId, float tMax)>;

        struct CompressedAABB
        {
            uint8_t low[3];
            uint8_t high[3];
        };

        void setLocalAABB(const math::AABB& localAABB);
        math::Vec3f getLocalScale() const;
        math::AABB getChildAABB(int childIndex) const;
        void setChildAABB(const math::AABB& childAABB, int childIndex);

        math::Vec3f localOrigin;
        uint8_t localScaleExp[3];
        uint8_t childLeafMask = 0;
        CompressedAABB childCompressedAABB[2];

        uint32_t childNdx[2] = {};
    };

    static_assert(sizeof(BranchNode) == 36);

    BranchNode* m_binTreeRoot{};
    std::vector<std::shared_ptr<MeshInstance>>* m_instances{};

    uint32_t generateHierarchy(
        const math::AABB* sortedLeafAABBs,
        uint32_t* sortedMortonCodes,
        uint32_t* sortedObjectIDs,
        int           first,
        int           last,
        math::AABB& treeBB);

    int findSplit(uint32_t* sortedMortonCodes,
        int           first,
        int           last);

    uint32_t allocBranch(uint32_t numNodes);
    uint32_t m_branchCount = 0;

    std::unique_ptr<BranchNode[]> m_internalNodes;
    math::AABB m_globalAABB;
};