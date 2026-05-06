#include "pch.h"
#include "CppUnitTest.h"
#include "..\MazeMap\MazeMask.h"
#include "sstream"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
	TEST_CLASS(MazeMaskTest)
	{
	public:

		TEST_METHOD(GetPerimeterTestHorizontal)
		{
			uint16_t baseData[16] =
			{
				0b0000'1111'1111'0000,
				0b0000'1111'1111'0000,
				0b0000'1111'1111'0000,
				0b0000'1111'1111'0000,

				0b0000'1111'1111'0000,
				0b0000'1111'1111'0000,
				0b0000'1111'1111'0000,
				0b0000'1111'1111'0000,

				0b0000'1111'1111'0000,
				0b0000'1111'1111'0000,
				0b0000'1111'1111'0000,
				0b0000'1111'1111'0000,

				0b0000'1111'1111'0000,
				0b0000'1111'1111'0000,
				0b0000'1111'1111'0000,
				0b0000'1111'1111'0000,
			};
			uint16_t expected[16] =
			{
				0b0000'1000'0001'0000,
				0b0000'1000'0001'0000,
				0b0000'1000'0001'0000,
				0b0000'1000'0001'0000,

				0b0000'1000'0001'0000,
				0b0000'1000'0001'0000,
				0b0000'1000'0001'0000,
				0b0000'1000'0001'0000,

				0b0000'1000'0001'0000,
				0b0000'1000'0001'0000,
				0b0000'1000'0001'0000,
				0b0000'1000'0001'0000,

				0b0000'1000'0001'0000,
				0b0000'1000'0001'0000,
				0b0000'1000'0001'0000,
				0b0000'1000'0001'0000,
			};

			MazeMask mask = MazeMask(baseData);
			MazeMask perimeter = mask.GetPerimeter(PerimeterType::Inner);

			for (uint8_t i = 0; i < 16; i++)
			{
				Assert::AreEqual(expected[i], perimeter.GetRow(i));
			}
		}
		TEST_METHOD(GetPerimeterTestVertical)
		{
			uint16_t baseData[16] =
			{
				0b0000'0000'0000'0000,
				0b0000'0000'0000'0000,
				0b0000'0000'0000'0000,
				0b0000'0000'0000'0000,

				0b1111'1111'1111'1111,
				0b1111'1111'1111'1111,
				0b1111'1111'1111'1111,
				0b1111'1111'1111'1111,

				0b1111'1111'1111'1111,
				0b1111'1111'1111'1111,
				0b1111'1111'1111'1111,
				0b1111'1111'1111'1111,
				
				0b0000'0000'0000'0000,
				0b0000'0000'0000'0000,
				0b0000'0000'0000'0000,
				0b0000'0000'0000'0000,
			};
			uint16_t expected[16] =
			{
				0b0000'0000'0000'0000,
				0b0000'0000'0000'0000,
				0b0000'0000'0000'0000,
				0b0000'0000'0000'0000,

				0b1111'1111'1111'1111,
				0b0000'0000'0000'0000,
				0b0000'0000'0000'0000,
				0b0000'0000'0000'0000,

				0b0000'0000'0000'0000,
				0b0000'0000'0000'0000,
				0b0000'0000'0000'0000,
				0b1111'1111'1111'1111,

				0b0000'0000'0000'0000,
				0b0000'0000'0000'0000,
				0b0000'0000'0000'0000,
				0b0000'0000'0000'0000,
			};

			MazeMask mask = MazeMask(baseData);
			MazeMask perimeter = mask.GetPerimeter(PerimeterType::Inner);
			std::wstringstream ss;
			for (uint8_t i = 0; i < 16; i++)
			{
				ss << "Row: " << i;

				Assert::AreEqual(expected[i], perimeter.GetRow(i), ss.str().c_str());

				ss.clear(std::ios_base::trunc);
			}
		}
		TEST_METHOD(GetPerimeterTestHorizontalOuter)
		{
			uint16_t baseData[16] =
			{
				0b0000'1111'1111'0000,
				0b0000'1111'1111'0000,
				0b0000'1111'1111'0000,
				0b0000'1111'1111'0000,

				0b0000'1111'1111'0000,
				0b0000'1111'1111'0000,
				0b0000'1111'1111'0000,
				0b0000'1111'1111'0000,

				0b0000'1111'1111'0000,
				0b0000'1111'1111'0000,
				0b0000'1111'1111'0000,
				0b0000'1111'1111'0000,

				0b0000'1111'1111'0000,
				0b0000'1111'1111'0000,
				0b0000'1111'1111'0000,
				0b0000'1111'1111'0000,
			};
			uint16_t expected[16] =
			{
				0b0001'0000'0000'1000,
				0b0001'0000'0000'1000,
				0b0001'0000'0000'1000,
				0b0001'0000'0000'1000,

				0b0001'0000'0000'1000,
				0b0001'0000'0000'1000,
				0b0001'0000'0000'1000,
				0b0001'0000'0000'1000,

				0b0001'0000'0000'1000,
				0b0001'0000'0000'1000,
				0b0001'0000'0000'1000,
				0b0001'0000'0000'1000,

				0b0001'0000'0000'1000,
				0b0001'0000'0000'1000,
				0b0001'0000'0000'1000,
				0b0001'0000'0000'1000,
			};

			MazeMask mask = MazeMask(baseData);
			MazeMask perimeter = mask.GetPerimeter(PerimeterType::Outer);

			for (uint8_t i = 0; i < 16; i++)
			{
				Assert::AreEqual(expected[i], perimeter.GetRow(i));
			}
		}
		TEST_METHOD(GetPerimeterTestVerticalOuter)
		{
			uint16_t baseData[16] =
			{
				0b0000'0000'0000'0000,
				0b0000'0000'0000'0000,
				0b0000'0000'0000'0000,
				0b0000'0000'0000'0000,

				0b1111'1111'1111'1111,
				0b1111'1111'1111'1111,
				0b1111'1111'1111'1111,
				0b1111'1111'1111'1111,

				0b1111'1111'1111'1111,
				0b1111'1111'1111'1111,
				0b1111'1111'1111'1111,
				0b1111'1111'1111'1111,

				0b0000'0000'0000'0000,
				0b0000'0000'0000'0000,
				0b0000'0000'0000'0000,
				0b0000'0000'0000'0000,
			};
			uint16_t expected[16] =
			{
				0b0000'0000'0000'0000,
				0b0000'0000'0000'0000,
				0b0000'0000'0000'0000,
				0b1111'1111'1111'1111,

				0b0000'0000'0000'0000,
				0b0000'0000'0000'0000,
				0b0000'0000'0000'0000,
				0b0000'0000'0000'0000,

				0b0000'0000'0000'0000,
				0b0000'0000'0000'0000,
				0b0000'0000'0000'0000,
				0b0000'0000'0000'0000,

				0b1111'1111'1111'1111,
				0b0000'0000'0000'0000,
				0b0000'0000'0000'0000,
				0b0000'0000'0000'0000,
			};

			MazeMask mask = MazeMask(baseData);
			MazeMask perimeter = mask.GetPerimeter(PerimeterType::Outer);
			std::wstringstream ss;
			for (uint8_t i = 0; i < 16; i++)
			{
				ss << "Row: " << i;

				Assert::AreEqual(expected[i], perimeter.GetRow(i), ss.str().c_str());

				ss.clear(std::ios_base::trunc);
			}
		}
		TEST_METHOD(PerimeterSpeedTest1M)
		{
			uint16_t baseData[16] =
			{
				0b0000'0000'0000'0000,
				0b0000'0000'0000'0000,
				0b0000'0000'0000'0000,
				0b0000'0000'0000'0000,

				0b1111'1111'1111'1111,
				0b1111'1111'1111'1111,
				0b1111'1111'1111'1111,
				0b1111'1111'1111'1111,

				0b1111'1111'1111'1111,
				0b1111'1111'1111'1111,
				0b1111'1111'1111'1111,
				0b1111'1111'1111'1111,

				0b0000'0000'0000'0000,
				0b0000'0000'0000'0000,
				0b0000'0000'0000'0000,
				0b0000'0000'0000'0000,
			};
			uint16_t expected[16] =
			{
				0b0000'0000'0000'0000,
				0b0000'0000'0000'0000,
				0b0000'0000'0000'0000,
				0b1111'1111'1111'1111,

				0b0000'0000'0000'0000,
				0b0000'0000'0000'0000,
				0b0000'0000'0000'0000,
				0b0000'0000'0000'0000,

				0b0000'0000'0000'0000,
				0b0000'0000'0000'0000,
				0b0000'0000'0000'0000,
				0b0000'0000'0000'0000,

				0b1111'1111'1111'1111,
				0b0000'0000'0000'0000,
				0b0000'0000'0000'0000,
				0b0000'0000'0000'0000,
			};

			MazeMask mask = MazeMask(baseData);
			MazeMask perimeter;
			for (int i = 0; i < 20000; ++i)
			{
				perimeter = mask.GetPerimeter(PerimeterType::Outer);
				perimeter = mask.GetPerimeter(PerimeterType::Outer);
				perimeter = mask.GetPerimeter(PerimeterType::Outer);
				perimeter = mask.GetPerimeter(PerimeterType::Outer);
				perimeter = mask.GetPerimeter(PerimeterType::Outer);

				perimeter = mask.GetPerimeter(PerimeterType::Outer);
				perimeter = mask.GetPerimeter(PerimeterType::Outer);
				perimeter = mask.GetPerimeter(PerimeterType::Outer);
				perimeter = mask.GetPerimeter(PerimeterType::Outer);
				perimeter = mask.GetPerimeter(PerimeterType::Outer);


				perimeter = mask.GetPerimeter(PerimeterType::Outer);
				perimeter = mask.GetPerimeter(PerimeterType::Outer);
				perimeter = mask.GetPerimeter(PerimeterType::Outer);
				perimeter = mask.GetPerimeter(PerimeterType::Outer);
				perimeter = mask.GetPerimeter(PerimeterType::Outer);

				perimeter = mask.GetPerimeter(PerimeterType::Outer);
				perimeter = mask.GetPerimeter(PerimeterType::Outer);
				perimeter = mask.GetPerimeter(PerimeterType::Outer);
				perimeter = mask.GetPerimeter(PerimeterType::Outer);
				perimeter = mask.GetPerimeter(PerimeterType::Outer);


				perimeter = mask.GetPerimeter(PerimeterType::Outer);
				perimeter = mask.GetPerimeter(PerimeterType::Outer);
				perimeter = mask.GetPerimeter(PerimeterType::Outer);
				perimeter = mask.GetPerimeter(PerimeterType::Outer);
				perimeter = mask.GetPerimeter(PerimeterType::Outer);

				perimeter = mask.GetPerimeter(PerimeterType::Outer);
				perimeter = mask.GetPerimeter(PerimeterType::Outer);
				perimeter = mask.GetPerimeter(PerimeterType::Outer);
				perimeter = mask.GetPerimeter(PerimeterType::Outer);
				perimeter = mask.GetPerimeter(PerimeterType::Outer);


				perimeter = mask.GetPerimeter(PerimeterType::Outer);
				perimeter = mask.GetPerimeter(PerimeterType::Outer);
				perimeter = mask.GetPerimeter(PerimeterType::Outer);
				perimeter = mask.GetPerimeter(PerimeterType::Outer);
				perimeter = mask.GetPerimeter(PerimeterType::Outer);

				perimeter = mask.GetPerimeter(PerimeterType::Outer);
				perimeter = mask.GetPerimeter(PerimeterType::Outer);
				perimeter = mask.GetPerimeter(PerimeterType::Outer);
				perimeter = mask.GetPerimeter(PerimeterType::Outer);
				perimeter = mask.GetPerimeter(PerimeterType::Outer);


				perimeter = mask.GetPerimeter(PerimeterType::Outer);
				perimeter = mask.GetPerimeter(PerimeterType::Outer);
				perimeter = mask.GetPerimeter(PerimeterType::Outer);
				perimeter = mask.GetPerimeter(PerimeterType::Outer);
				perimeter = mask.GetPerimeter(PerimeterType::Outer);

				perimeter = mask.GetPerimeter(PerimeterType::Outer);
				perimeter = mask.GetPerimeter(PerimeterType::Outer);
				perimeter = mask.GetPerimeter(PerimeterType::Outer);
				perimeter = mask.GetPerimeter(PerimeterType::Outer);
				perimeter = mask.GetPerimeter(PerimeterType::Outer);
			}
			std::wstringstream ss;
			for (uint8_t i = 0; i < 16; i++)
			{
				ss << "Row: " << i;

				Assert::AreEqual(expected[i], perimeter.GetRow(i), ss.str().c_str());

				ss.clear(std::ios_base::trunc);
			}
		}

	};
}
