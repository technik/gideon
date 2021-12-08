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

#include <cassert>
#include <memory>
#include <functional>
#include <vector>

#include <math/matrix.h>
#include <math/vector.h>
#include <math/aabb.h>

namespace math
{
    class Ray;
}

class MeshInstance;
class BLAS;
struct HitRecord;

// Compressed wide BVH based on Karras 2017
class CWBVH
{
public:
    CWBVH();
    ~CWBVH();
    void build(
        const std::vector<math::AABB>& aabbs);
    auto aabb() const { return m_globalAABB; }

    template<class LeafOp>
    inline bool traceRay(
        const math::Ray::Implicit& ray,
        float tMax,
        const LeafOp& leafCallback) const;

    class TraversalState;

    bool hitClosest(
        const math::Ray&,
        float tMax,
        HitRecord& collision,
        const std::vector<std::shared_ptr<MeshInstance>>& instances) const;

    struct Instance
    {
        math::Matrix34f pose;
        uint32_t BlasIndex;
    };

    bool hitClosest(
        const math::Ray&,
        float tMax,
        HitRecord& collision,
        BLAS* blasBuffer,
        Instance* instances,
        uint32_t numInstances) const;

    bool continueTraverse(
        TraversalState& stack,
        uint32_t& hitId
    ) const;
    //bool hitAny(const math::Ray&, float tMax);

    class TraversalState
    {
    public:
        // Point stack to the root of the tree
        void reset(math::Ray::Implicit& _r, float _tMax)
        {
            // Init ray
            r = _r;
            tMax = _tMax;
            // Reset stack
            stack[0] = 0;
            top = &stack[1];
        }

        bool empty() const { return stack == top; }

        void push(uint32_t nodeId)
        {
            // Store nodeId together with the index of the next branch that needs parsing,
            // which is always 0 when pushing a brand new node.
            assert(((nodeId << 1) >> 1) == nodeId);
            *top = nodeId << 1;
            ++top;
        }

        uint32_t pop() {
            auto nodeAndChildNdx = *(top - 1);
            if ((nodeAndChildNdx & 1) > 0) // This is the second and last time we pop this node
                --top;
            else
                (*(top - 1))++; // Prepare for next iteration
            assert(top >= stack);
            return nodeAndChildNdx;
        }

        math::Ray::Implicit r;
        float tMax;

    private:
        static constexpr uint32_t kMaxStackSize = 40;
        uint32_t stack[kMaxStackSize];
        uint32_t* top = stack;
    };

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
    //std::vector<std::shared_ptr<MeshInstance>>* m_instances{};

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

// Inline implementation
template<class LeafOp>
inline bool CWBVH::traceRay(
    const math::Ray::Implicit& r,
    float tMax,
    const LeafOp& leafCallback) const
{
    assert(m_binTreeRoot != nullptr);

    // Check against global aabb. This should be moved outside for Two level AS to be efficient.
    if (!m_globalAABB.intersect(r, tMax))
        return false;

    // Init traversal stack to the root
    TraversalStack stack;
    stack.reset();
    float t = -1;

    while (!stack.empty())
    {
        const auto& branch = m_internalNodes[stack.pop()];

        for (int i = 0; i < 2; ++i)
        {
            if (branch.getChildAABB(i).intersect(r, tMax))
            {
                if (branch.childLeafMask & (1 << i)) // Child is a leaf, perform leaf test
                {
                    if(float tHit = leafCallback(branch.childNdx[i], tMax); tHit >= 0)
                    {
                        hitId = branch.childNdx[i];
                        t = tHit;
                        tMax = t;
                    }
                }
                else // Child is a branch. Add it to the stack
                {
                    stack.push(branch.childNdx[i]);
                }
            }
        }
    }

    return t >= 0;
}