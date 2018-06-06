//-------------------------------------------------------------------------------------------------
// Based on the minibook 'Raytracing in one weekend' and Aras P.'s series: Daily pathtracer
// https://aras-p.info/blog/
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

class MeshInstance : public Shape
{
public:
	MeshInstance(const std::shared_ptr<MultiMesh>& mesh, const math::Matrix34f& transform)
		: mMesh(mesh)
	{
		mXForm = transform;
		mXFormInv = transform.inverse();
	}

	bool hit(const math::Ray & r, float tMin, float tMax, HitRecord & collision) const override
	{
		math::Ray localRay (mXFormInv.transformPos(r.origin()), mXFormInv.transformDir(r.direction()));

		if(mMesh->hit(localRay, tMin, tMax, collision))
		{
			collision.normal = mXForm.transformDir(collision.normal);
			collision.p = mXForm.transformPos(collision.p);
			tMax = collision.t;
			return true;
		}

		return false;
	}
private:
	std::shared_ptr<const MultiMesh> mMesh;
	math::Matrix34f mXForm;
	math::Matrix34f mXFormInv;
};
