#include "pch.h"
#include "CppUnitTest.h"
#include "../RTBase/Core.h"
#include "../RTBase/Geometry.h"
#include <iostream>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace RTtest
{
	TEST_CLASS(RTtest)
	{
	public:
		
		TEST_METHOD(TestMethod1)
		{
			Vec3 a(1.0f, 2.0f, 3.0f);
			Vec3 b(2.0f, 1.0f, 1.0f);
			Vec3 c = a + b;
			Assert::AreEqual(c.x, 3.0f, 0.0001f);
		}
		TEST_METHOD(TestMethod_RayIntersectPlane1)
		{
			Vec3 o(0.0f, 0.0f, 0.0f);
			Vec3 dir(1.0f, 1.0f, 1.0f);
			Ray r(o, dir);

			Vec3 n(0.0f, 1.0f, 0.0f);
			float d = 1; //p0 = (0,1,0) d=-(0*0+1*1+0*0)
			Plane p;
			p.init(n, d);

			float t;
			Assert::IsTrue(p.rayIntersect(r,t));
		}
		TEST_METHOD(TestMethod_RayIntersectPlane2)
		{
			Vec3 o(0.0f, 2.0f, 0.0f);
			Vec3 dir(1.0f, 1.0f, 1.0f);
			Ray r(o, dir);

			Vec3 n(0.0f, 1.0f, 0.0f);
			float d = 1; //p0 = (0,1,0) d=-(0*0+1*1+0*0)
			Plane p;
			p.init(n, d);

			float t;
			Assert::IsFalse(p.rayIntersect(r, t));
		}
		TEST_METHOD(RayAABB) {
			Vec3 o(0.0f, 0.0f, 0.0f);
			Vec3 dir(1.0f, 1.0f, 1.0f);
			Ray r(o, dir);

			AABB box;
			box.extend(Vec3(0, 0, 0));
			box.extend(Vec3(1, 1, 1));

			float t;
			Assert::IsTrue(box.rayAABB(r,t));
		}
		TEST_METHOD(RaySphere1) {
			Vec3 o(0.0f, 0.0f, 0.0f);
			Vec3 dir(1.0f, 1.0f, 1.0f);
			Ray r(o, dir);

			Vec3 centre(0.0f, 0.0f, 0.0f);
			float radius = 1.0f;
			Sphere s;
			s.init(centre, radius);

			float t;
			Assert::IsTrue(s.rayIntersect(r,t));
			Assert::AreEqual(t, 1.0f, 0.0001f);
		}
		TEST_METHOD(RaySphere2) {
			Vec3 o(2.0f, 2.0f, 2.0f);
			Vec3 dir(1.0f, 1.0f, 1.0f);
			Ray r(o, dir);

			Vec3 centre(0.0f, 0.0f, 0.0f);
			float radius = 1.0f;
			Sphere s;
			s.init(centre, radius);

			float t;
			Assert::IsFalse(s.rayIntersect(r, t));
			
		}
		TEST_METHOD(RaySphere3) {
			Vec3 o(-3.0f, -3.0f, -3.0f);
			Vec3 dir(1.0f, 1.0f, 1.0f);
			Ray r(o, dir);

			Vec3 centre(0.0f, 0.0f, 0.0f);
			float radius = 1.0f;
			Sphere s;
			s.init(centre, radius);

			float t;
			Assert::IsTrue(s.rayIntersect(r, t));
			Assert::AreEqual(t, 1.0f, 0.0001f);
		}
	};
}
