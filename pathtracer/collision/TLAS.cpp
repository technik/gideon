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
    
    for (auto& instance : instances)
    {
        aabbs.push_back(instance.pose * blasBuffer[instance.BlasIndex].aabb());
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

    auto hitInfo = m_bvh.closestHit(ray, tMax, [this,&closestT,&closestNormal](const math::Ray& ray, float tMax, uint32_t& closestHitId) {
        const auto& instance = m_instances[closestHitId];

        // Transform the ray to local coordinates
        math::Ray localRay;
        localRay.origin() = instance.pose.transformPos(ray.origin());
        localRay.direction() = instance.pose.transformDir(ray.direction());

        // Intersect ray with the BLAS
        auto& blas = m_BLASBuffer[instance.BlasIndex];
        float tHit;
        math::Vec3f hitNormal;
        uint32_t closestHitTriId;
        if (blas.closestHit(localRay, tMax, closestHitTriId, tHit, hitNormal))
        {
            closestT = tHit;
            closestNormal = hitNormal;
            return tHit;
        }

        return -1.f;
    });

    if (hitInfo.empty())
        return false;

    dst.normal = closestNormal;
    dst.p = ray.at(closestT);
    dst.t = closestT;

    return true;
}