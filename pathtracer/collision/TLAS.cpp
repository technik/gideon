#include "TLAS.h"

#include "BLAS.h"

//--------------------------------------------------------------------------------------------------
void TLAS::build(std::shared_ptr <BLAS[]> blasBuffer, std::shared_ptr<Instance[]> instances, uint32_t numInstances)
{
    std::vector<math::AABB> aabbs(numInstances);
    for (uint32_t i = 0; i < numInstances; ++i)
    {
        auto& instance = instances[i];
        aabbs[i] = instance.pose * blasBuffer[instance.BlasIndex].aabb();;
    }

    m_BLASBuffer = std::move(blasBuffer);
    m_instances = std::move(instances);
}

//--------------------------------------------------------------------------------------------------
bool TLAS::closestHit(const math::Ray& ray, float tMax) const
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
        float tHit;
        math::Vec3f hitNormal;
        if (m_BLASBuffer[instance.BlasIndex].closestHit(localRay, stack.tMax, triHitId, tHit, hitNormal))
        {
            // TODO: Compute intersection details
            assert(false && "Not implemented"); return false; // Return false for obvious break
            return true;
        }
    }

    return false;
}