#include "pch.h"
#include "CppUnitTest.h"
#include "../RTBase/Core.h"

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
	};
}
