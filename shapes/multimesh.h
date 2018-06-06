#pragma once

#include "shape.h"
#include "triangleMesh.h"

class MultiMesh : Shape
{
public:
	MultiMesh(const std::vector<TriangleMesh>& mesh, const std::vector<std::shared_ptr<PBRMaterial>>& mat)
		: mMeshes(mesh)
		, mMaterials(mat)
	{}

	bool hit(const math::Ray & r, float tMin, float tMax, HitRecord & collision) const override
	{
		bool hit_any = false;
		for(size_t i = 0; i < mMeshes.size(); ++i)
		{
			if(mMeshes[i].hit(r, tMin, tMax, collision))
			{
				collision.material = mMaterials[i].get();
				tMax = collision.t;
				hit_any = true;
			}
		}
		return hit_any;
	}

	const std::vector<TriangleMesh> mMeshes;
	const std::vector<std::shared_ptr<PBRMaterial>> mMaterials;
};
