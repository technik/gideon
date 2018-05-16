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

#include "camera.h"
#include <math/ray.h>
#include <math/vector3.h>

class FrustumCamera : public Camera
{
public:
	FrustumCamera(
		const math::Vec3f& pos,
		const math::Vec3f& lookAt,
		float horFov,
		size_t winX,
		size_t winY
	)
		: origin(pos)
	{
		auto depth = normalize(lookAt - pos);
		auto side = normalize(cross(depth, {0.f,1.f,0.f}));
		auto up = cross(side, depth);
		auto hLen = std::tan(horFov/2.f);
		auto vLen = winY*hLen/winX;
		ll_corner = depth - hLen * side - vLen * up;
		horizontal = 2*hLen*side;
		vertical = 2*vLen*up;
	}

	math::Ray get_ray(float u, float v) const override
	{
		return math::Ray(
			origin,
			ll_corner + u*horizontal + v*vertical
		);
	}

private:
	math::Vec3f ll_corner;
	math::Vec3f horizontal;
	math::Vec3f vertical;
	math::Vec3f origin;
};