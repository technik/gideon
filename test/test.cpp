// test.cpp : Defines the entry point for the console application.
//
#include <cassert>
//#include <textures/textureSampler.h>

#include <math/vector.h>
//#include <math/matrix.h>
//#include <math/quaterrnion.h>
//#include <math/aabb.h>
#include <math/constants.h>
//#include <math/vectorFloat.h>
//#include <random.h>

/*void testWrappers()
{
	// Test repeat
	{
		RepeatWrap wrap(4);
		// Basic range
		assert(wrap(0.f) == wrap(1.f) == wrap(2.f) == 0);
		assert(wrap(0.99f) == 3);
		assert(wrap(0.5f) == 2);
		// Past one
		assert(wrap(0.4f) == wrap(1.4f) == 1);
		// Negative range
		assert(wrap(-1.f) == 0);
		assert(wrap(-2.f) == 0);
		assert(wrap(-0.5f) == wrap(0.5f));
		assert(wrap(-2.5f) == wrap(0.5f));
		assert(wrap(-0.4f) == 2);
		assert(wrap(-2.4f) == wrap(0.6f));
	}
	// Test clamp
	{
		ClampWrap wrap(4);
		// Basic range
		assert(wrap(0.f)  == 0);
		assert(wrap(0.99f) == 3);
		assert(wrap(1.f) == 3);
		assert(wrap(0.5f) == 2);
		// Past one
		assert(wrap(1.1f) == 3);
		assert(wrap(3.4f) == 3);
		// Negative range
		assert(wrap(-1.f) == wrap(0.f));
		assert(wrap(-2.f) == wrap(0.f));
		assert(wrap(-0.5f) == wrap(0.f));
		assert(wrap(-2.5f) == wrap(0.f));
		assert(wrap(-2.4f) == wrap(0.f));
	}
}*/

//using math::float4;

void testVector()
{
	using math::Vec2f;
	using math::Vec3f;
	using math::Vec4f;
	static_assert(Vec2f::rows == 2 && Vec2f::cols == 1);
	static_assert(Vec3f::rows == 3 && Vec3f::cols == 1);
	static_assert(Vec4f::rows == 4 && Vec4f::cols == 1);

	constexpr math::UniformExpr<float,3,3> constant(42);
	static_assert(constant[3] == 42.f);

	{
		// Test initializations
		Vec3f a;
		Vec3f b(1.f);
		assert(b.x() == 1.f);
		assert(b.y() == 1.f);
		assert(b.z() == 1.f);
		Vec3f c(2);
		assert(c.x() == 2.f);
		assert(c.y() == 2.f);
		assert(c.z() == 2.f);
		Vec3f d(3.f, 4.f);
		assert(d.x() == 3.f);
		assert(d.y() == 4.f);
		assert(d.z() == 0.f);
		// Test basic operators
		a = b+c;
		assert(a.x() = 3.f);
		d = -a;
		assert(d.x() == -3.f);
	}

	// test binary operators
	{
		auto a = Vec3f(1.f);
		auto b = Vec3f(2.f);
		Vec3f c = a+b;
		assert(c.x() == 3.f);
	}

	// Test matrix views for insertion

	/*{
		float4 a(1.f,2.f,3.f,4.f);
		assert(a.x() == 1.f);
		float4 b = a.shuffle<0,0,0,0>();
		float4 c = a.shuffle<1,1,1,1>();
		float4 d = a.shuffle<2,3,0,0>();
		float4 e = a.shuffle<1,0,0,0>();
		float4 f = a.shuffle<3,0,1,2>();
		float4 h = a.shuffle<3,0,1,2>();
	}
	{
		auto x = float4(1.f);
		auto y = float4(5.f);
		auto c = x+y;
		assert(c.x() == 6.f);
	}
	{
		math::AABBSimd b1(math::Vec3f(0.f), math::Vec3f(1.f));
		math::Ray r0 = { math::Vec3f(-1.f), normalize(math::Vec3f(1.f)) };

		float t;
		assert(!b1.intersect(r0.implicitSimd(), 0.f, 1.f, t));
		assert(b1.intersect(r0.implicitSimd(), 0.f, 2.f, t));

		math::Ray r1 = { math::Vec3f(-0.5f, 0.5f, 0.5f), normalize(math::Vec3f(1.f, 0.f, 0.f)) };
		assert(b1.intersect(r1.implicitSimd(), 0.f, 1.f, t));
		assert(!b1.intersect(r1.implicitSimd(), 0.f, 0.1f, t));

		math::Ray r2 = { math::Vec3f(-0.5f, 0.5f, 0.5f), normalize(math::Vec3f(0.f, 1.f, 0.f)) };
		assert(!b1.intersect(r2.implicitSimd(), 0.f, 10.f, t));

		// TODO: Add tests for the intersections shown in GDC2018 - Extreme SIMD talk.
	}*/
}

/*

using math::Matrix44f;
using math::Matrix34f;

void testMatrix()
{
	// Test LU factorization
	{ // TODO
	}
	// Test matrix inverse
	{
		auto m = Matrix44f::identity();
		auto mi = m.inverse();
		assert(m == mi);
	}
	{
		constexpr int nRot = 10;
		constexpr int nTrans = 10;
		RandomGenerator g;
		for(int i = 0; i < 3; ++i)
		{
			auto scale = math::Vec3f(g.scalar(), g.scalar(), g.scalar()) + math::Vec3f(1.f);
			auto scaleMatrix = Matrix34f::identity();
			scaleMatrix(0,0) = scale.x();
			scaleMatrix(1,1) = scale.y();
			scaleMatrix(2,2) = scale.z();
			for(int j = 0; j < nRot; ++j)
			{
				auto angle = j * math::TwoPi / nRot;
				auto q = math::Quatf(g.unit_vector(), angle);
				auto rot = q.rotationMtx();
				for(auto k = 0; k < nTrans; ++k)
				{
					auto trans = float(k) * g.unit_vector();
					auto m = rot * scaleMatrix;
					m.position() = trans;
					auto mi = m.inverse();

					for(auto l = 0; l < 5; ++l)
					{
						auto v = g.unit_vector();
						auto vp = m.transformDir(v);
						auto vr = mi.transformDir(vp);
						auto align = dot(v, vr);
						assert(align > 0.99f);
						assert(align < 1.01f);
					}
				}
			}
		}
	}
}*/

int main()
{
	testVector();
	//testMatrix();
	//testWrappers();
    return 0;
}