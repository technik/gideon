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
#include <span>

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
    void build(std::span<const math::AABB> aabbs);
    auto aabb() const { return m_globalAABB; }

    class TraversalState;

    // Deprecated. Use the traversal state API, or the LeafOp API instead.
    bool hitClosest(
        const math::Ray&,
        float tMax,
        HitRecord& collision,
        const std::vector<std::shared_ptr<MeshInstance>>& instances) const;

    struct HitInfo
    {
        bool empty() const { return mNodeId >= 0; }
        int32_t mNodeId = -1;
        float t = -1.f;
    };

    struct Instance
    {
        math::Matrix34f pose;
        uint32_t BlasIndex;
    };

    // Leaf Op takes a node index (in the order provided at build time),
    // and returns a boolean: true when traversal can be finished early (e.g. collision found),
    // false otherwise.
    template<class LeafOp>
    bool anyHit(const math::Ray& ray, float tMax, const LeafOp& leafOp) const
    {
        // Check against global aabb
        auto implicitRay = ray.implicit();
        if (!m_globalAABB.intersect(implicitRay, tMax))
            return false;

        // Init traversal stack to the root
        CWBVH::TraversalState stack;
        stack.reset(implicitRay, tMax);

        uint32_t instanceHitId;
        while (continueTraverse(stack, instanceHitId))
        {
            if (leafOp(instanceHitId))
                return true;
        }

        // Exhausted traversal
        return false;
    }

    // Leaf Op takes a ray, max distance and a node index (in the order provided at build time),
    // and returns an intersection distance, or -1 if no intersection was found.
    template<class LeafOp>
    HitInfo closestHit(const math::Ray& ray, float tMax, const LeafOp& leafOp) const
    {
        HitInfo hitInfo;
        // Check against global aabb
        auto implicitRay = ray.implicit();
        if (!m_globalAABB.intersect(implicitRay, tMax))
            return hitInfo;

        // Init traversal stack to the root
        CWBVH::TraversalState stack;
        stack.reset(implicitRay, tMax);

        int32_t closestHit = -1;
        uint32_t instanceHitId;
        while (continueTraverse(stack, instanceHitId))
        {
            float tHit = leafOp(ray, stack.tMax, instanceHitId);
            if (tHit >= 0) // Intersection found, reduce testing distance
            {
                closestHit = instanceHitId;
                stack.tMax = std::min(stack.tMax, tHit);
            }
        }

        // Exhausted traversal
        if (closestHit >= 0)
        {
            hitInfo.mNodeId = closestHit;
            hitInfo.t = stack.tMax;
        }

        return hitInfo;
    }

    // Deprecated. Use the traversal state API, or the LeafOp API instead.
    bool hitClosest(
        const math::Ray&,
        float tMax,
        HitRecord& collision,
        const BLAS* blasBuffer,
        const Instance* instances,
        const math::Matrix34f* invPoses,
        uint32_t numInstances) const;

    bool continueTraverse(
        TraversalState& stack,
        uint32_t& hitId
    ) const;

    class TraversalState
    {
    public:
        // Point stack to the root of the tree
        void reset(const math::Ray::Implicit& _r, float _tMax)
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
    void createSingleLeafHierarchy(const math::AABB& leaf);
    uint32_t m_branchCount = 0;

    std::shared_ptr<BranchNode[]> m_internalNodes;
    math::AABB m_globalAABB;
};