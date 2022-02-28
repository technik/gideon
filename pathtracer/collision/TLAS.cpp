#include "TLAS.h"

#include "BLAS.h"

//--------------------------------------------------------------------------------------------------
void TLAS::build(
    std::vector<BLAS>&& blasBuffer,
    std::vector<Instance>&& instances)
{
    // Transform instance bboxes to the common frame of reference
    std::vector<math::AABB> aabbs;
    aabbs.reserve(instances.size());
    m_invInstancePoses.reserve(instances.size());
    
    for (auto& instance : instances)
    {
        aabbs.push_back(instance.pose * blasBuffer[instance.BlasIndex].aabb());
        m_invInstancePoses.push_back(instance.pose.inverse());
    }

    // Keep references
    m_BLASBuffer = std::move(blasBuffer);
    m_instances = std::move(instances);

    // Build the TLAS bvh
    m_bvh.build(aabbs);
}

//--------------------------------------------------------------------------------------------------
bool TLAS::closestHit(const math::Ray& ray, float tMax, HitRecord& dst) const
{
    // Check against global aabb
    auto implicitRay = ray.implicit();
    if (!m_bvh.aabb().intersect(implicitRay, tMax))
        return false;

    float closestT = std::numeric_limits<float>::max();
    math::Vec3f closestNormal;

    auto blasTest = [this, &closestT, &closestNormal](const math::Ray& globalRay, float tMax, uint32_t& closestHitId) {
        // Transform the ray to local coordinates
        const auto& invPose = m_invInstancePoses[closestHitId];
        math::Ray localRay;
        localRay.origin() = invPose.transformPos(globalRay.origin());
        localRay.direction() = invPose.transformDir(globalRay.direction());

        // Intersect ray with the BLAS
        const auto& instance = m_instances[closestHitId];
        auto& blas = m_BLASBuffer[instance.BlasIndex];
        float tHit;
        math::Vec3f hitNormal;
        uint32_t closestHitTriId = -1;
        if (blas.closestHit(localRay, tMax, closestHitTriId, tHit, hitNormal))
        {
            closestT = tHit;
            closestNormal = instance.pose.transformDir(hitNormal);
            return tHit;
        }

        return -1.f;
    };

    auto hitInfo = m_bvh.closestHit(ray, tMax, blasTest);

    if (hitInfo.empty())
        return false;

    dst.normal = closestNormal;
    dst.p = ray.at(closestT);
    dst.t = closestT;

    return true;
}