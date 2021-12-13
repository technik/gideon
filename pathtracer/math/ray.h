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

#include "vector.h"
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

                Vec3f& origin() { return mOrigin; }
        const   Vec3f& origin() const { return mOrigin; }
		        Vec3f& direction() { return mDirection; }
		const   Vec3f& direction() const { return mDirection; }
		Vec3f at(float t) const { return mOrigin + t * mDirection; }

		// Implicit ray
		struct Implicit {
			Vec3f o; // -origin/dir
			Vec3f n; // 1 / dir
		};

		struct ImplicitSimd
		{
			float4 o; // -origin/dir
			float4 n; // 1 / dir
		};

		struct Simd // Ray replicated in 4 simd lanes
		{
			Vec3f4 o;
			Vec3f4 d;
		};

		Simd simd() const {
			return {
				math::Vec3f4( mOrigin.x(), mOrigin.y(), mOrigin.z() ),
				math::Vec3f4( mDirection.x(), mDirection.y(), mDirection.z() )
			};
		}

		// Compute an implicit representation of the ray. Useful for batched intersection tests.
		Implicit implicit() const
		{
			Vec3f invDir(
				1.f / mDirection.x(),
				1.f / mDirection.y(),
				1.f / mDirection.z()
			);
			return Implicit{mOrigin, invDir};
		}

		ImplicitSimd implicitSimd() const
		{
			float4 origin = float4(mOrigin);
			Vec3f invDir = {
				1.f / mDirection.x(),
				1.f / mDirection.y(),
				1.f / mDirection.z()
			};

			// The trick of storing tMin and tMax inside the implicit vector
			// saves shuffle instructions, but requires tMax and tMin's last component to
			// be properly setup
			return ImplicitSimd{ origin, float4(invDir) };
		}

	private:
		Vec3f mOrigin;
		Vec3f mDirection;
	};
}
