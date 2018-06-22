#pragma once

#include <cassert>
#include <cmath>
#include "constants.h"

namespace math {

	int factorial(int x)
	{
		if(x <= 1)
			return 1;
		return x*factorial(x-1);
	}

	float shNorm(int l, int m)
	{
		auto num = (2*l+1)*factorial(l-abs(m));
		auto den = 4*math::Constants<float>::pi*factorial(l+abs(m));
		return sqrt(num/den);
	}

	float legendre(int l, int m, float sinTheta, float z)
	{
		assert(m <= l);
		if(m == 0 && l == 0)
		{
			return 1.f;
		}
		else
		{
			float zp = z*pow(sinTheta,m);
			if(m == l)
			{
				return (1-2*m)*legendre(l-1,m-1,sinTheta,z);
			}
			else if(l == m+1)
			{
				return (2*m+1)*zp*legendre(m,m,sinTheta, z);
			} else
			{
				auto num = (2*l-1)*zp*legendre(l-1,m,sinTheta, z) - (l+m-1)*legendre(l-2,m,sinTheta,z);
				auto den = l-m;
				return num/den;
			}
		}
	}

	//--------------------------------------------------------------------------------------------------
	float sh(int l, int m, float cosTheta, float phi)
	{
		float sinTheta = sqrt(1-cosTheta*cosTheta);
		if(m > 0)
		{
			return sqrt(2.f)*shNorm(l,m)*cos(m*phi)*legendre(l,m,sinTheta,cosTheta);
		}else if(m < 0)
		{
			return sqrt(2.f)*shNorm(l,m)*sin(abs(m)*phi)*legendre(l,abs(m),sinTheta,cosTheta);
		}
		else
			return shNorm(l,0)*legendre(l,0,sinTheta,cosTheta);
	}

	// Spherical harmonics coefficients
	/*auto dir =r.direction();
	dir.normalize();
	auto phi = atan2(dir.y(),dir.x());
	auto cTheta = dir.z();
	auto shVal = sh(4,1,cTheta,phi);
	Vec3f c;
	c.r() = max(0.f,shVal);
	c.g() = abs(shVal)>1.001f?1.f:0.f;
	c.b() = max(0.f,-shVal);
	return c;*/

}	// namespace math
