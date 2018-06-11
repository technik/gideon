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
#include <math/vector.h>
#include <math/constants.h>

class SphericalCamera : public Camera
{
public:
	SphericalCamera(
		const math::Vec3f& _pos,
		const math::Vec3f& _lookAt,
		const math::Vec3f& _up
	)
		: origin(_pos)
	{
		fwd = normalize((_lookAt - _pos));
		side = normalize(cross(fwd, _up));
		up = cross(side, fwd);
	}

	math::Ray get_ray(float u, float v) const override
	{
		auto phi = (math::Constants<float>::twoPi * u);
		auto theta = (math::Constants<float>::pi * v);
		auto cosTheta = std::cos(theta);
		auto sinTheta = std::sin(theta);
		auto cosPhi = std::cos(phi);
		auto sinPhi = std::sin(phi);
		return math::Ray(
			origin,
			math::Vec3f(sinTheta*cosPhi,sinTheta*sinPhi,-cosTheta)
			/*-cosTheta*up +
			sinTheta*cosPhi*fwd+
			sinTheta*sinPhi*side*/
		);
	}

private:
	math::Vec3f origin;
	math::Vec3f up;
	math::Vec3f fwd;
	math::Vec3f side;
};