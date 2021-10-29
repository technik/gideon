#include "CWBVH.h"

#include <math/aabb.h>
#include <math/ray.h>
#include <shapes/meshInstance.h>
#include <math/vectorFloat.h>

#include <numeric>

template<uint32_t numSpaces, uint32_t numBits>
uint32_t spaceBits(uint32_t x)
{
    uint32_t srcBitMask = 1;
    uint32_t dstBitMask = 1;
    uint32_t result = x & srcBitMask;

    for (uint32_t i = 1; i < numBits; ++i)
    {
        srcBitMask <<= 1;
        dstBitMask <<= 1 + numSpaces;
        result += (x & srcBitMask) ? dstBitMask : 0;
    }

    return result;
}

// TODO: test this version against above code
unsigned int expandBits(unsigned int v)
{
    v = (v * 0x00010001u) & 0xFF0000FFu;
    v = (v * 0x00000101u) & 0x0F00F00Fu;
    v = (v * 0x00000011u) & 0xC30C30C3u;
    v = (v * 0x00000005u) & 0x49249249u;
    return v;
}

struct CWBVH::Node
{
    Node(const math::AABB& aabb, bool leaf = false)
        : isLeaf(leaf)
        , m_aabb(aabb)
    {}

    using BlasCallback = std::function<float(uint32_t leafId, float tMax)>;

    virtual float hitClosest(math::Ray::Implicit& r, float tMax, uint32_t& hitId, const BlasCallback&) = 0;
    bool isLeaf = false;
    math::AABB m_aabb;
};

struct CWBVH::LeafNode : Node
{
    LeafNode() : Node(math::AABB()) {}
    LeafNode(const math::AABB& aabb, uint32_t id)
        : Node(aabb, true)
        , leafId(id)
    {}

    float hitClosest(math::Ray::Implicit& r, float tMax, uint32_t& hitId, const BlasCallback& cb) override
    {
        //float tBox;
        //if (m_aabb.intersect(r, tMax, tBox))
        {
            float tHit = cb(leafId, tMax);
            if (tHit >= 0)
            {
                hitId = leafId;
                return tHit;
            }
        }
        return -1;
    }
    uint32_t leafId;
};

struct CWBVH::BranchNode : Node
{
    BranchNode() : Node(math::AABB()) {}

    BranchNode(Node* a, Node* b)
        : Node(math::AABB(a->m_aabb, b->m_aabb))
        , childA(a)
        , childB(b)
    {
    }

    float hitClosest(math::Ray::Implicit& r, float tMax, uint32_t& hitId, const BlasCallback& cb) override
    {
        float maxEnter;
        if (!m_aabb.intersect(r, tMax, maxEnter))
            return -1;

        float t = -1;
        if (childA)
        {
            t = childA->hitClosest(r, tMax, hitId, cb);
            if (t > -1)
                tMax = t;
        }
        if (childB)
        {
            float tb = childB->hitClosest(r, tMax, hitId, cb);
            if (tb > -1)
                t = tb;
        }
        return t;
    }

    Node* childA = nullptr;
    Node* childB = nullptr;
};

int CWBVH::findSplit(uint32_t* sortedMortonCodes,
    int           first,
    int           last)
{
    // Identical Morton codes => split the range in the middle.

    unsigned int firstCode = sortedMortonCodes[first];
    unsigned int lastCode = sortedMortonCodes[last];

    if (firstCode == lastCode)
        return (first + last) >> 1;

    // Calculate the number of highest bits that are the same
    // for all objects, using the count-leading-zeros intrinsic.
    int commonPrefix = __lzcnt(firstCode ^ lastCode);

    // Use binary search to find where the next bit differs.
    // Specifically, we are looking for the highest object that
    // shares more than commonPrefix bits with the first one.

    int split = first; // initial guess
    int step = last - first;

    do
    {
        step = (step + 1) >> 1; // exponential decrease
        int newSplit = split + step; // proposed new position

        if (newSplit < last)
        {
            unsigned int splitCode = sortedMortonCodes[newSplit];
            int splitPrefix = __lzcnt(firstCode ^ splitCode);
            if (splitPrefix > commonPrefix)
                split = newSplit; // accept proposal
        }
    }     while (step > 1);

    return split;
}

CWBVH::Node* CWBVH::generateHierarchy(
    const math::AABB* sortedLeafAABBs,
    uint32_t* sortedMortonCodes,
    uint32_t* sortedObjectIDs,
    int           first,
    int           last)
{
    // Single object => create a leaf node.
    if (first == last)
        return new LeafNode(sortedLeafAABBs[first], sortedObjectIDs[first]);

    // Determine where to split the range.
    int split = findSplit(sortedMortonCodes, first, last);

    // Process the resulting sub-ranges recursively.

    Node* childA = generateHierarchy(sortedLeafAABBs, sortedMortonCodes, sortedObjectIDs,
        first, split);
    Node* childB = generateHierarchy(sortedLeafAABBs, sortedMortonCodes, sortedObjectIDs,
        split + 1, last);
    return new BranchNode(childA, childB);
}

void CWBVH::build(std::vector<std::shared_ptr<MeshInstance>>& instances)
{
    m_instances = &instances;

    // Find the absolute bounding box of all triangles
    // and store their centers
    // TODO: Maybe extend the bounding box to the centers only for improved quantization precision
    std::vector<math::Vec3f> centers;
    centers.reserve(instances.size());
    math::AABB globalAABB;
    globalAABB.clear();
    for (auto& t : instances)
    {
        auto instanceBB = t->aabb();
        globalAABB.add(instanceBB.min());
        globalAABB.add(instanceBB.max());

        centers.push_back(instanceBB.origin());
    }
    math::Vec3f invGlobalAABBSize = math::Vec3f(1.f,1.f,1.f) / globalAABB.size();

    // Assign morton code quadrants to each centroid
    std::vector<uint32_t> mortonSections;
    mortonSections.reserve(instances.size());
    for (auto& trianglePos : centers)
    {
        math::Vec3f normalizedPos = (trianglePos - globalAABB.min()) * invGlobalAABBSize;

        // Quantize position. 11 bits x, 11 bits y, 10 bits z.
        uint32_t quantX = std::min<uint32_t>(normalizedPos.x() * (1 << 11), (1 << 11) - 1);
        uint32_t quantY = std::min<uint32_t>(normalizedPos.y() * (1 << 11), (1 << 11) - 1);
        uint32_t quantZ = std::min<uint32_t>(normalizedPos.z() * (1 << 10), (1 << 10) - 1);

        // Interlace morton codes
        uint32_t mortonCode = spaceBits<2, 11>(quantX) | (spaceBits<2, 11>(quantY)<<1) | (spaceBits<2, 10>(quantZ)<<2);
        mortonSections.push_back(mortonCode);
    }

    // Sort triangles based on their morton codes
    std::vector<uint32_t> indices(instances.size());
    std::iota(indices.begin(), indices.end(), 0);
    std::sort(indices.begin(), indices.end(), [&](auto a, auto b) {
        return mortonSections[a] < mortonSections[b];
        });
    std::vector<uint32_t> sortedMortonCodes(instances.size());
    std::vector<math::AABB> sortedLeafAABBs(instances.size());
    for (size_t i = 0; i < instances.size(); ++i)
    {
        auto ndx = indices[i];
        sortedMortonCodes[i] = mortonSections[ndx];
        sortedLeafAABBs[i] = instances[ndx]->aabb();
    }

    // Allocate enough nodes to hold the tree
    m_leafs = std::unique_ptr<LeafNode[]>(new LeafNode[instances.size()]());
    m_internalNodes = std::unique_ptr<BranchNode[]>(new BranchNode[instances.size()-1]());

    // Build a binary tree out of the sorted nodes
    m_binTreeRoot = generateHierarchy(
        sortedLeafAABBs.data(),
        sortedMortonCodes.data(),
        indices.data(),
        0,
        instances.size() - 1);
}

bool CWBVH::hitClosest(
    const math::Ray& ray,
    float tMax,
    HitRecord& collision) const
{
    if (!m_binTreeRoot)
        return false;

    math::Ray::Implicit r = ray.implicit();
    uint32_t hitId;
    auto cb = [this,&ray,&collision](uint32_t leafId, float tMax)
    {
        HitRecord hitInfo;
        if((*m_instances)[leafId]->hit(ray, tMax, hitInfo))
        {
            collision = hitInfo;
            return hitInfo.t;
        }
        return -1.f;
    };
    float tHit = m_binTreeRoot->hitClosest(r, tMax, hitId, cb);

    return tHit >= 0;
}
CWBVH::LeafNode& CWBVH::allocLeaf()
{
    return m_leafs[m_leafCount++];
}

CWBVH::BranchNode& CWBVH::allocBranch()
{
    return m_internalNodes[m_branchCount++];
}