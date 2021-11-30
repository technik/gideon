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
#include "ray.h"
#include "linear.h"

namespace math
{
	/// Axis aligned bounding box
	struct AABB {
		using Vector = Vec3f;

		// Constructors
		AABB() = default;
		AABB(const Vector& _min, const Vector& _max)
			: mMin(_min), mMax(_max)
		{ }

		AABB(const Vector& _o, float _size)
			:mMin(_o-_size*0.5f*Vector(1.f,1.f,1.f))
			,mMax(_o+_size*0.5f*Vector(1.f,1.f,1.f))
		{ }

		AABB(const AABB& a, const AABB& b)
			: mMin(math::min(a.min(),b.min()))
			, mMax(math::max(a.max(),b.max()))
		{
		}

		float area() const {
			auto h = mMax-mMin;
			return 2.f*(h.x()*h.y()+h.x()*h.z()+h.y()*h.z());
		}

		// Size, position and volume
		/// Make the Box empty
		void clear() {
			mMin = Vector(std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity());
			mMax = Vector(-std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity());
		}

		bool empty() const {
			return mMax.x() < mMin.x()
				|| mMax.y() < mMin.y()
				|| mMax.z() < mMin.z();
		}

		void add(const Vector& v)
		{
			mMin = math::min(mMin, v);
			mMax = math::max(v, mMax);
		}

		const Vector& min() const { return mMin; }
		const Vector& max() const { return mMax; }
		Vector size() const { return mMax - mMin; }
		Vector origin() const { return 0.5f*(mMin + mMax); }

		bool contains(const Vector& _point) const {
			return (math::min(_point, mMin) == mMin) && (math::max(_point, mMax) == mMax);
		}

		// Intersection and distance
		/// find intersection between this box and a ray, in the ray's parametric interval [_tmin, _tmax]
		/// Also, store the minimun intersection distance into _tout
		bool intersect(const Ray::Implicit& _r, float _tmin, float _tmax, float& _maxEnter) const {
			Vector t1 = (mMin -_r.o)*_r.n;
			Vector t2 = (mMax -_r.o)*_r.n;
			// Swapping the order of comparison is important because of NaN behavior
			auto tEnter = math::min(t1,t2);
			auto tLeave = math::max(t2,t1);
			_maxEnter = math::max(tEnter.x(), math::max(tEnter.y(), math::max(tEnter.z(), _tmin)));
			auto minLeave = math::min(tLeave.x(), math::min(tLeave.y(), math::min(tLeave.z(), _tmax)));
			return minLeave >= _maxEnter;
		}

		// Intersection and distance
		/// find intersection between this box and a ray, in the ray's parametric interval [_tmin, _tmax]
		/// Also, store the minimun intersection distance into _tout
		bool intersect(const Ray::Implicit& _r, float _tmax, float& _maxEnter) const {
			Vector t1 = (mMin -_r.o)*_r.n;
			Vector t2 = (mMax -_r.o)*_r.n;
			// Swapping the order of comparison is important because of NaN behavior
			auto tEnter = math::min(t1,t2);
			auto tLeave = math::max(t2,t1);
			_maxEnter = math::max(tEnter.x(), math::max(tEnter.y(), math::max(tEnter.z(), 0.f)));
			auto minLeave = math::min(tLeave.x(), math::min(tLeave.y(), math::min(tLeave.z(), _tmax)));
			return minLeave >= _maxEnter;
		}

        // Intersection and distance
        /// find intersection between this box and a ray, in the ray's parametric interval [_tmin, _tmax]
        bool intersect(const Ray::Implicit& _r, float _tmax) const {
            Vector t1 = (mMin - _r.o) * _r.n;
            Vector t2 = (mMax - _r.o) * _r.n;
            // Swapping the order of comparison is important because of NaN behavior
            auto tEnter = math::min(t1, t2);
            auto tLeave = math::max(t2, t1);
            auto maxEnter = math::max(tEnter.x(), math::max(tEnter.y(), math::max(tEnter.z(), 0.f)));
            auto minLeave = math::min(tLeave.x(), math::min(tLeave.y(), math::min(tLeave.z(), _tmax)));
            return minLeave >= maxEnter;
        }
	private:
		Vector mMin;
		Vector mMax;
	};

	/// Axis aligned bounding box, SIMD version
	struct AABBSimd {
		using Vector = float4;

		// Constructors
		AABBSimd() = default;
		AABBSimd(const Vec3f& _min, const Vec3f& _max)
			: mMin(_min.x(),_min.y(),_min.z(),_min.z())
			, mMax(_max.x(),_max.y(),_max.z(),_max.z())
		{ }

		AABBSimd(const AABBSimd& a, const AABBSimd& b)
			: mMin(math::min(a.mMin,b.mMin))
			, mMax(math::max(a.mMax,b.mMax))
		{
		}

		// Size, position and volume
		/// Make the Box empty
		void clear() {
			mMin = Vector(_mm_set_ps1(std::numeric_limits<float>::infinity()));
			mMax = Vector(_mm_set_ps1(-std::numeric_limits<float>::infinity()));
		}

		bool empty() const {
			return float4(mMax <= mMin).any();
		}

		void add(const Vector& v)
		{
			mMin = math::min(mMin, v);
			mMax = math::max(mMax, v);
		}

		const Vector& min() const { return mMin; }
		const Vector& max() const { return mMax; }
		Vector size() const { return mMax - mMin; }

		/// find intersection between this box and a ray, in the ray's parametric interval [_tmin, _tmax]
		/// Also, store the minimun intersection distance into _tout
		bool intersect(const Ray::ImplicitSimd& _r, float4 _tmax, float& _tCollide) const {
			Vector t1 = (mMin - _r.o) * _r.n;
			Vector t2 = (mMax - _r.o) * _r.n;
			// Swapping the order of comparison is important because of NaN behavior and SSE
			auto tEnter = math::min(t2,t1); // Enters
			auto tExit = math::max(t1,t2); // Exits
			auto maxEnter = math::max(tEnter,float4(0.f)); // If nan, return second operand, which is never nan
			auto minLeave = math::min(tExit,_tmax); // If nan, return second operand, which is never nan
			_tCollide = maxEnter.hMax(); // Furthest enter
			return minLeave.hMin() >= _tCollide;
		}
	private:
		Vector mMin;
		Vector mMax;
	};
}