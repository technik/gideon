//-------------------------------------------------------------------------------------------------
// Toy path tracer
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

#include "vector3.h"
#include "vectorFloat.h"

namespace math
{
	class Ray
	{
	public:
		Ray() = default;
		Ray( const Vec3f& ro ///< Ray origin
			,const Vec3f& rd ///< Ray direction
		)
			: mOrigin(ro)
			, mDirection(rd)
		{};

		const Vec3f& origin() const { return mOrigin; }
		const Vec3f& direction() const { return mDirection; }
		Vec3f at(float t) const { return mOrigin + t * mDirection; }

		// Implicit ray
		struct Implicit {
			Vec3f o; // -origin/dir
			Vec3f n; // 1 / dir
		};

		struct ImplicitSimd
		{
			VecSimd3f o; // -origin/dir
			VecSimd3f n; // 1 / dir
		};

		// Compute an implicit representation of the ray. Useful for batched intersection tests.
		Implicit implicit() const {
			Vec3f invDir(
				1.f / mDirection.x(),
				1.f / mDirection.y(),
				1.f / mDirection.z()
			);
			Vec3f origin(
				mDirection.x() ? -mOrigin.x() * invDir.x() : 0.f,
				mDirection.y() ? -mOrigin.y() * invDir.y() : 0.f,
				mDirection.z() ? -mOrigin.z() * invDir.z() : 0.f
			);
			return Implicit{origin, invDir};
		}

		ImplicitSimd implicitSimd() const {

		}

	private:
		Vec3f mOrigin;
		Vec3f mDirection;
	};
}
