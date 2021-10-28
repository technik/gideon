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
#include <vector>

#include <math/vector.h>

namespace math
{
    class Ray;
    struct AABB;
}

class MeshInstance;
struct HitRecord;

// Compressed wide BVH based on Karras 2017
class CWBVH
{
public:
    void build(std::vector<std::shared_ptr<MeshInstance>>& instances);

    //bool hitAny(const math::Ray&, float tMax);
    bool hitClosest(
        const math::Ray&,
        float tMax,
        HitRecord& collision) const;


private:
    class Node;
    class LeafNode;
    class BranchNode;

    Node* m_binTreeRoot;
    std::vector<std::shared_ptr<MeshInstance>>* m_instances;

    Node* generateHierarchy(
        const math::AABB* sortedLeafAABBs,
        uint32_t* sortedMortonCodes,
        uint32_t* sortedObjectIDs,
        int           first,
        int           last);

    int findSplit(uint32_t* sortedMortonCodes,
        int           first,
        int           last);

    struct InternalNode
    {
        math::Vec3f origin;
        uint8_t scaleExp[3]; // Exponent bits of quantized bbox size
        uint8_t imask;
        uint32_t childBaseIndex;
        uint32_t leafBaseIndex;
        // Compressed children SOA
        uint8_t meta[8];
        uint8_t qLoX[8];
        uint8_t qLoY[8];
        uint8_t qLoZ[8];
        uint8_t qHiX[8];
        uint8_t qHiY[8];
        uint8_t qHiZ[8];
    };

    static_assert(sizeof(InternalNode) == 80);
};