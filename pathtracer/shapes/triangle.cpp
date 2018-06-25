#include "triangle.h"
#include "collision.h"

bool Triangle::hit(
	const math::Ray& r,
	float tMin,
	float tMax,
	HitRecord& collision
) const
{
	auto p0 = r.at(tMin);

	auto h0 = v[0]-p0;
	auto h1 = v[1]-p0;
	auto h2 = v[2]-p0;

	auto rd = r.direction();
	auto a0 = cross(h0,h1);
	auto a1 = cross(h1,h2);
	auto a2 = cross(h2,h0);

	if((dot(a0,rd) < 0.f) && (dot(a1,rd) < 0.f) && (dot(a2,rd) < 0.f))
	{
		auto offset0 = dot(p0, mNormal);

		float t = tMin + (mPlaneOffset-offset0)/dot(rd, mNormal);
		if( t >= tMin && t < tMax)
		{
			auto p = r.at(t);

			collision.t = t;
			collision.p = p;
			collision.normal = mNormal;

			return true;
		}
	}

	return false;
}