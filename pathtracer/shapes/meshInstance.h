//-------------------------------------------------------------------------------------------------
// Toy path tracer
//--------------------------------------------------------------------------------------------------
// Copyright 2018 Carmelo J Fdez-Aguera
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

#include "triangleMesh.h"
#include <math/matrix.h>
#include <math/ray.h>
#include <memory>
#include "shape.h"

class MeshInstance
{
public:
	MeshInstance(const std::shared_ptr<Shape>& mesh, const math::Matrix34f& x)
		: mMesh(mesh)
	{
		mXForm = x;
		mXFormInv = x.inverse();

		auto det = 
			x(0,0) * (x(1,1)*x(2,2)-x(2,1)*x(1,2)) -
			x(0,1) * (x(1,0)*x(2,2)-x(2,0)*x(1,2)) +
			x(0,2) * (x(1,0)*x(2,1)-x(2,0)*x(1,1));
		mXFormScaleSign = det > 0? 1.f : -1.f;
	}

	bool hit(const math::Ray & r, float tMax, HitRecord & collision) const
	{
		auto localDir = mXFormInv.transformDir(r.direction());
		//auto norm = 1 / std::sqrt(dot(localDir, localDir));
		//localDir *= norm;
		math::Ray localRay (mXFormInv.transformPos(r.origin()), localDir);

        float tout;
		if(mMesh->bbox().intersect(localRay.implicit(), tMax, tout))
		{
			if(mMesh->hit(localRay, tMax, collision))
			{
				collision.normal = mXForm.transformDir(mXFormScaleSign * collision.normal);
				collision.p = mXForm.transformPos(collision.p);
			//	collision.t *= norm;
				return true;
			}
		}

		return false;
	}

private:
	std::shared_ptr<const Shape> mMesh;
	math::Matrix34f mXForm;
	math::Matrix34f mXFormInv;
	float mXFormScaleSign;
};
