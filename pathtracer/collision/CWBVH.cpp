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

float floatFromExponent(int8_t e);

// Returns the log2(p) where p is the smallest power of two such that p > abs(x).
// note that p can be < 0, in cases where 0<abs(x)<1.
// Assumes non denormal floats.
int8_t nextPow2Log2(float x)
{
    auto bitField = reinterpret_cast<uint32_t&>(x);
    auto e = int8_t((bitField >> 23) + 1);
    return e;
}

float floatFromExponent(int8_t e)
{
    uint32_t bitField = uint32_t(uint8_t(e)) << 23;
    auto x = reinterpret_cast<float&>(bitField);
    assert(nextPow2Log2(x) - 1 == e);
    return x;
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

void CWBVH::BranchNode::setLocalAABB(const math::AABB& localAABB)
{
    localOrigin = localAABB.min();
    auto extent = localAABB.size();
    localScaleExp[0] = nextPow2Log2(extent.x());
    localScaleExp[1] = nextPow2Log2(extent.y());
    localScaleExp[2] = nextPow2Log2(extent.z());
}

math::Vec3f CWBVH::BranchNode::getLocalScale() const
{
    return math::Vec3f(
        floatFromExponent(localScaleExp[0]),
        floatFromExponent(localScaleExp[1]),
        floatFromExponent(localScaleExp[2])
    );
}

math::AABB CWBVH::BranchNode::getChildAABB(int childIndex) const
{
    const auto& compressed = childCompressedAABB[childIndex];
    // recover size
    math::Vec3f parentExtent = getLocalScale();
    math::Vec3f low = localOrigin;
    low.x() += compressed.low[0] * parentExtent.x() / 255;
    low.y() += compressed.low[1] * parentExtent.y() / 255;
    low.z() += compressed.low[2] * parentExtent.z() / 255;

    math::Vec3f high = localOrigin;
    high.x() += compressed.high[0] * parentExtent.x() / 255;
    high.y() += compressed.high[1] * parentExtent.y() / 255;
    high.z() += compressed.high[2] * parentExtent.z() / 255;

    return math::AABB(low, high);
}

void CWBVH::BranchNode::setChildAABB(const math::AABB& childAABB, int childIndex)
{
    math::Vec3f relMin = childAABB.min() - localOrigin;
    math::Vec3f relMax = childAABB.max() - localOrigin;
    // Normalize range
    auto localExtent = getLocalScale();
    auto normMin = relMin / localExtent;
    auto normMax = relMax / localExtent;

    // Quantize coordinates
    CompressedAABB aabb;
    aabb.low[0] = uint8_t(normMin.x() * 255);
    aabb.low[1] = uint8_t(normMin.y() * 255);
    aabb.low[2] = uint8_t(normMin.z() * 255);

    aabb.high[0] = (uint8_t)std::min((normMax.x() * 255)+1, 255.f);
    aabb.high[1] = (uint8_t)std::min((normMax.y() * 255)+1, 255.f);
    aabb.high[2] = (uint8_t)std::min((normMax.z() * 255)+1, 255.f);

    // Store compressed in the index
    childCompressedAABB[childIndex] = aabb;
}

// Out of line constructor for smart pointers
CWBVH::CWBVH()
{}

// Out of line deleter for smart pointers
CWBVH::~CWBVH()
{}

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

uint32_t CWBVH::generateHierarchy(
    const math::AABB* sortedLeafAABBs,
    uint32_t* sortedMortonCodes,
    uint32_t* sortedObjectIDs,
    int           first,
    int           last,
    math::AABB& treeBB
)
{
    // Single object => create a leaf node.
    assert(first != last && "Leafs are supposed to be solved in the parent node");

    auto branchNdx = allocBranch(1);
    // Determine where to split the range.
    int split = findSplit(sortedMortonCodes, first, last);

    // Process the resulting sub-ranges recursively.
    math::AABB bboxA, bboxB;
    auto branch = &m_internalNodes[branchNdx];

    if (first == split)
    {
        bboxA = sortedLeafAABBs[first];
        branch->childLeafMask |= 1;
        branch->childNdx[0] = sortedObjectIDs[first];
    }
    else
    {
        branch->childNdx[0] = generateHierarchy(sortedLeafAABBs, sortedMortonCodes, sortedObjectIDs,
            first, split, bboxA);
    }

    if (split + 1 == last)
    {
        bboxB = sortedLeafAABBs[last];
        branch->childLeafMask |= 2;
        branch->childNdx[1] = sortedObjectIDs[last];
    }
    else
    {
        branch->childNdx[1] = generateHierarchy(sortedLeafAABBs, sortedMortonCodes, sortedObjectIDs,
            split + 1, last, bboxB);
    }

    treeBB = math::AABB(bboxA, bboxB);
    branch->setLocalAABB(treeBB);
    branch->setChildAABB(bboxA, 0);
    branch->setChildAABB(bboxB, 1);
    return branchNdx;
}

void CWBVH::build(std::vector<std::shared_ptr<MeshInstance>>& instances)
{
    m_instances = &instances;

    // Find the absolute bounding box of all triangles
    // and store their centers
    // TODO: Maybe extend the bounding box to the centers only for improved quantization precision
    std::vector<math::Vec3f> centers;
    centers.reserve(instances.size());
    
    m_globalAABB.clear();
    for (auto& t : instances)
    {
        auto instanceBB = t->aabb();
        m_globalAABB.add(instanceBB.min());
        m_globalAABB.add(instanceBB.max());

        centers.push_back(instanceBB.origin());
    }
    math::Vec3f invGlobalAABBSize = math::Vec3f(1.f,1.f,1.f) / m_globalAABB.size();

    // Assign morton code quadrants to each centroid
    std::vector<uint32_t> mortonSections;
    mortonSections.reserve(instances.size());
    for (auto& trianglePos : centers)
    {
        math::Vec3f normalizedPos = (trianglePos - m_globalAABB.min()) * invGlobalAABBSize;

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
    m_internalNodes = std::unique_ptr<BranchNode[]>(new BranchNode[instances.size()-1]());

    // Build a binary tree out of the sorted nodes
    math::AABB treeAABB;
    auto binTreeRootId = generateHierarchy(
        sortedLeafAABBs.data(),
        sortedMortonCodes.data(),
        indices.data(),
        0,
        instances.size() - 1, treeAABB);
    m_binTreeRoot = &m_internalNodes[binTreeRootId];
}

bool CWBVH::hitClosest(
    std::vector<uint32_t>& stack,
    const math::Ray& ray,
    float tMax,
    HitRecord& collision) const
{
    if (!m_binTreeRoot)
        return false;

    // Prepare implicit ray for fast intersection tests
    math::Ray::Implicit r = ray.implicit();
    float maxEnter;
    // Check against global aabb
    if (!m_globalAABB.intersect(r, tMax, maxEnter))
        return false;

    // Leaf callback
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

    // Init traversal stack to the root
    stack.clear();
    stack.push_back(0);
    float t = -1;

    while (!stack.empty())
    {
        const auto& branch = m_internalNodes[stack.back()];
        stack.pop_back();

        for (int i = 0; i < 2; ++i)
        {
            float maxEnter;
            if (branch.getChildAABB(i).intersect(r, tMax, maxEnter))
            {
                if (branch.childLeafMask & (1 << i)) // Child is a leaf, perform leaf test
                {
                    float tHit = cb(branch.childNdx[i], tMax);
                    if (tHit >= 0)
                    {
                        hitId = branch.childNdx[i];
                        t = tHit;
                        tMax = t;
                    }
                }
                else // Child is a branch. Add it to the stack
                {
                    stack.push_back(branch.childNdx[i]);
                }
            }
        }
    }

    return t >= 0;
}

uint32_t CWBVH::allocBranch(uint32_t numNodes)
{
    auto nextNode = m_branchCount;
    m_branchCount += numNodes;
    return nextNode;
}