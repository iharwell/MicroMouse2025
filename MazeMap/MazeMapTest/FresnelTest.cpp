#include "pch.h"
#include "CppUnitTest.h"
#include "Templates.h"
#include "..\MazeMap\Fresnel.h"
#include <chrono>
#include <sstream>
#include <vector>
#include <iomanip>
using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
	TEST_CLASS(FresnelTest)
	{
	public:

		void GetError(double inputValue, double& sError, double& cError)
		{
			double inputd = inputValue;
			double sd = 0.0;
			double cd = 0.0;

			float inputf = static_cast<float>(inputValue);
			float sf = 0.0f;
			float cf = 0.0f;

			fresnel(inputd, cd, sd);
			fresnelf(inputf, cf, sf);

			sError = fabs(sd - sf) / sd;
			cError = fabs(cd - cf) / cd;
		}

		void GetError(double inputValue, double& sError, double& cError, double extraParam)
		{
			double inputd = inputValue;
			double sd = 0.0;
			double cd = 0.0;

			float inputf = static_cast<float>(inputValue);
			float sf = 0.0f;
			float cf = 0.0f;

			fresnel(inputd, cd, sd);
			fresnelf_test(inputf, cf, sf, static_cast<float>(extraParam));

			sError = fabs(sd - sf) / sd;
			cError = fabs(cd - cf) / cd;
		}

		void TestValue(double value)
		{
			double inputd = value;
			double sd = 0.0;
			double cd = 0.0;

			float inputf = static_cast<float>(value);
			float sf = 0.0f;
			float cf = 0.0f;

			fresnel(inputd, cd, sd);
			fresnelf(inputf, cf, sf);

			double sdelta = fabs(sd - sf) / sd;
			double cdelta = fabs(cd - cf) / cd;

			Assert::IsTrue(sdelta < 0.0001, L"Fresnel S has too much error.");
			Assert::IsTrue(cdelta < 0.0001, L"Fresnel C has too much error.");

			std::stringstream ss("");
			ss << "S error: " << (sdelta * 100) << "%    C error: " << (cdelta * 100) << "%";
			std::string str = ss.str();
			Microsoft::VisualStudio::CppUnitTestFramework::Logger::WriteMessage(str.c_str());
		}

		TEST_METHOD(TestMethod1)
		{
			TestValue(1.0);
		}
		TEST_METHOD(TestMethod2)
		{
			TestValue(2.0);
		}
		TEST_METHOD(TestMethod3)
		{
			TestValue(3.0);
		}
		TEST_METHOD(TestMethod4)
		{
			TestValue(4.0);
		}
		TEST_METHOD(TestMethod5)
		{
			TestValue(5.0);
		}

		void PerfTest1()
		{
			std::stringstream ss("");
			float inputf = 0.0f;
			float sf = 0.0f;
			float cf = 0.0f;

			auto start = std::chrono::high_resolution_clock::now();
			for (int i = 0; i < 1100000; i++)
			{
				for (int j = 0; j < 250; j += 5)
				{
					fresnelf(0.01f * static_cast<float>(j), cf, sf);
					fresnelf(0.01f * static_cast<float>(j + 1), cf, sf);
					fresnelf(0.01f * static_cast<float>(j + 2), cf, sf);
					fresnelf(0.01f * static_cast<float>(j + 3), cf, sf);
					fresnelf(0.01f * static_cast<float>(j + 4), cf, sf);
				}
			}


			auto end = std::chrono::high_resolution_clock::now();
			std::chrono::nanoseconds duration = end - start;
			ss << (duration.count() * 0.000000001) << "\n";
			std::string str = ss.str();
			Microsoft::VisualStudio::CppUnitTestFramework::Logger::WriteMessage(str.c_str());
		}
		void PerfTest1(int startPoint, int span)
		{
			std::stringstream ss("");
			float inputf = 0.0f;
			float sf = 0.0f;
			float cf = 0.0f;

			auto start = std::chrono::high_resolution_clock::now();
			for (int i = 0; i < 1100000; i++)
			{
				for (int j = startPoint; j < startPoint + span; j += 5)
				{
					fresnelf(0.01f * static_cast<float>(j), cf, sf);
					fresnelf(0.01f * static_cast<float>(j + 1), cf, sf);
					fresnelf(0.01f * static_cast<float>(j + 2), cf, sf);
					fresnelf(0.01f * static_cast<float>(j + 3), cf, sf);
					fresnelf(0.01f * static_cast<float>(j + 4), cf, sf);
				}
			}


			auto end = std::chrono::high_resolution_clock::now();
			std::chrono::nanoseconds duration = end - start;
			ss << (duration.count() * 0.000000001) << "\n";
			std::string str = ss.str();
			Microsoft::VisualStudio::CppUnitTestFramework::Logger::WriteMessage(str.c_str());
		}

		TEST_METHOD(TestSet)
		{
			double maxErrorS = 0.0;
			double maxErrorC = 0.0;

			double maxErrorSInput = 0.0;
			double maxErrorCInput = 0.0;

			for (size_t i = 0; i < 250; i++)
			{
				double inputd = 0.01 * i;
				double sd = 0.0;
				double cd = 0.0;

				float inputf = static_cast<float>(inputd);
				float sf = 0.0f;
				float cf = 0.0f;

				fresnel(inputd, cd, sd);
				fresnelf(inputf, cf, sf);

				double sdelta = fabs(sd - sf) / sd;
				double cdelta = fabs(cd - cf) / cd;

				if (maxErrorS < sdelta)
				{
					maxErrorS = sdelta;
					maxErrorSInput = inputd;
				}
				if (maxErrorC < cdelta)
				{
					maxErrorC = cdelta;
					maxErrorCInput = inputd;
				}

			}
			Assert::IsTrue(maxErrorS < 0.001, L"Fresnel S has too much error.");
			Assert::IsTrue(maxErrorC < 0.001, L"Fresnel C has too much error.");

			std::stringstream ss("");
			ss << "S error: " << (maxErrorS * 100) << "%    Max S Error at: " << maxErrorSInput << "\n";
			ss << "C error: " << (maxErrorC * 100) << "%    Max C Error at: " << maxErrorCInput << "\n";
			//ss << "Max S Error at: " << maxErrorSInput << "      Max C Error at: " << maxErrorCInput << "\n";

			std::string str = ss.str();
			Microsoft::VisualStudio::CppUnitTestFramework::Logger::WriteMessage(str.c_str());
			/*for (int i = 150; i < 250; i += 5)
			{
				PerfTest1(i, 5);
			}*/

			float inputf = 0.0f;
			float sf = 0.0f;
			float cf = 0.0f;

			auto start = std::chrono::high_resolution_clock::now();

			for (int i = 0; i < 110000; i++)
			{
				for (int j = 0; j < 250; j += 5)
				{
					fresnelf(0.01f * static_cast<float>(j), cf, sf);
					fresnelf(0.01f * static_cast<float>(j + 1), cf, sf);
					fresnelf(0.01f * static_cast<float>(j + 2), cf, sf);
					fresnelf(0.01f * static_cast<float>(j + 3), cf, sf);
					fresnelf(0.01f * static_cast<float>(j + 4), cf, sf);
				}
			}


			auto end = std::chrono::high_resolution_clock::now();
			std::chrono::nanoseconds duration = end - start;
			ss << (duration.count() * 0.000000001);
		}
		TEST_METHOD(TestSet2)
		{
			double maxErrorS = 0.0;
			double maxErrorC = 0.0;

			double maxErrorSInput = 0.0;
			double maxErrorCInput = 0.0;

			for (size_t i = 25000; i < 45000; i++)
			{
				double inputd = 0.0001 * i;
				double sd = 0.0;
				double cd = 0.0;

				float inputf = static_cast<float>(inputd);
				float sf = 0.0f;
				float cf = 0.0f;

				fresnel(inputd, cd, sd);
				fresnelf(inputf, cf, sf);

				double sdelta = fabs(sd - sf) / sd;
				double cdelta = fabs(cd - cf) / cd;

				if (maxErrorS < sdelta)
				{
					maxErrorS = sdelta;
					maxErrorSInput = inputd;
				}
				if (maxErrorC < cdelta)
				{
					maxErrorC = cdelta;
					maxErrorCInput = inputd;
				}

			}
			//Assert::IsTrue(maxErrorS < 0.001, L"Fresnel S has too much error.");
			//Assert::IsTrue(maxErrorC < 0.001, L"Fresnel C has too much error.");

			std::stringstream ss("");
			ss << "S error: " << (maxErrorS * 100) << "%    Max S Error at: " << maxErrorSInput << "\n";
			ss << "C error: " << (maxErrorC * 100) << "%    Max C Error at: " << maxErrorCInput << "\n";
			//ss << "Max S Error at: " << maxErrorSInput << "      Max C Error at: " << maxErrorCInput << "\n";

			float inputf = 0.0f;
			float sf = 0.0f;
			float cf = 0.0f;

			auto start = std::chrono::high_resolution_clock::now();

			for (int i = 0; i < 137500; i++)
			{
				for (int j = 250; j < 450; j += 5)
				{
					fresnelf(0.01f * static_cast<float>(j), cf, sf);
					fresnelf(0.01f * static_cast<float>(j + 1), cf, sf);
					fresnelf(0.01f * static_cast<float>(j + 2), cf, sf);
					fresnelf(0.01f * static_cast<float>(j + 3), cf, sf);
					fresnelf(0.01f * static_cast<float>(j + 4), cf, sf);
				}
			}


			auto end = std::chrono::high_resolution_clock::now();
			std::chrono::nanoseconds duration = end - start;
			ss << (duration.count() * 0.000000001);
			std::string str = ss.str();
			Microsoft::VisualStudio::CppUnitTestFramework::Logger::WriteMessage(str.c_str());
		}
		TEST_METHOD(TestSet3)
		{
			double maxErrorS = 0.0;
			double maxErrorC = 0.0;

			double maxErrorSInput = 0.0;
			double maxErrorCInput = 0.0;

			for (size_t i = 450; i < 1000; i++)
			{
				double inputd = 0.01 * i;
				double sd = 0.0;
				double cd = 0.0;

				float inputf = static_cast<float>(inputd);
				float sf = 0.0f;
				float cf = 0.0f;

				fresnel(inputd, cd, sd);
				fresnelf(inputf, cf, sf);

				double sdelta = fabs(sd - sf) / sd;
				double cdelta = fabs(cd - cf) / cd;

				if (maxErrorS < sdelta)
				{
					maxErrorS = sdelta;
					maxErrorSInput = inputd;
				}
				if (maxErrorC < cdelta)
				{
					maxErrorC = cdelta;
					maxErrorCInput = inputd;
				}

			}
			Assert::IsTrue(maxErrorS < 0.001, L"Fresnel S has too much error.");
			Assert::IsTrue(maxErrorC < 0.001, L"Fresnel C has too much error.");

			std::stringstream ss("");
			ss << "S error: " << (maxErrorS * 100) << "%    Max S Error at: " << maxErrorSInput << "\n";
			ss << "C error: " << (maxErrorC * 100) << "%    Max C Error at: " << maxErrorCInput << "\n";
			//ss << "Max S Error at: " << maxErrorSInput << "      Max C Error at: " << maxErrorCInput << "\n";

			float inputf = 0.0f;
			float sf = 0.0f;
			float cf = 0.0f;

			auto start = std::chrono::high_resolution_clock::now();

			for (int i = 0; i < 50000; i++)
			{
				for (int j = 450; j < 1000; j += 5)
				{
					fresnelf(0.1f * static_cast<float>(j), cf, sf);
					fresnelf(0.1f * static_cast<float>(j + 1), cf, sf);
					fresnelf(0.1f * static_cast<float>(j + 2), cf, sf);
					fresnelf(0.1f * static_cast<float>(j + 3), cf, sf);
					fresnelf(0.1f * static_cast<float>(j + 4), cf, sf);
				}
			}


			auto end = std::chrono::high_resolution_clock::now();
			std::chrono::nanoseconds duration = end - start;
			ss << (duration.count() * 0.000000001);
			std::string str = ss.str();
			Microsoft::VisualStudio::CppUnitTestFramework::Logger::WriteMessage(str.c_str());
		}

		TEST_METHOD(TestSearch)
		{
			double minErrorS = 100;
			double minErrorC = 100;

			double minErrorSInput = 0.0;
			double minErrorCInput = 0.0;

			double minErrorSparam = 1.0e-20;
			double minErrorCparam = 1.0e-20;

			double Testparam = 1.0e-40;

			while (Testparam < 1.0e-7)
			{

				for (int i = 250; i < 450; i++)
				{
					double inputd = 0.01 * i;
					double sd = 0.0;
					double cd = 0.0;

					float inputf = static_cast<float>(inputd);
					float sf = 0.0f;
					float cf = 0.0f;

					fresnel(inputd, cd, sd);
					fresnelf_test(inputf, cf, sf, static_cast<float>(Testparam));

					double sdelta = fabs(sd - sf) / sd;
					double cdelta = fabs(cd - cf) / cd;

					if (minErrorS > sdelta)
					{
						minErrorS = sdelta;
						minErrorSInput = inputd;
						minErrorSparam = Testparam;
					}
					if (minErrorC > cdelta)
					{
						minErrorC = cdelta;
						minErrorCInput = inputd;
						minErrorCparam = Testparam;
					}

				}
				Testparam *= sqrt(10);
			}

			std::stringstream ss("");
			ss << "S error: " << (minErrorS * 100) << "%    Min S Error at: " << minErrorSInput << "\n" << "Param: " << minErrorSparam << "\n";
			ss << "C error: " << (minErrorC * 100) << "%    Min C Error at: " << minErrorCInput << "\n" << "Param: " << minErrorCparam << "\n";
			//ss << "Max S Error at: " << maxErrorSInput << "      Max C Error at: " << maxErrorCInput << "\n";

			/*float inputf = 0.0f;
			float sf = 0.0f;
			float cf = 0.0f;

			auto start = std::chrono::high_resolution_clock::now();

			for (size_t i = 0; i < 100000; i++)
			{
				for (size_t j = 450; j < 1000; j += 5)
				{
					fresnelf(0.1f * j, cf, sf);
					fresnelf(0.1f * (j + 1), cf, sf);
					fresnelf(0.1f * (j + 2), cf, sf);
					fresnelf(0.1f * (j + 3), cf, sf);
					fresnelf(0.1f * (j + 4), cf, sf);
				}
			}


			auto end = std::chrono::high_resolution_clock::now();
			std::chrono::nanoseconds duration = end - start;
			ss << (duration.count() * 0.000000001);*/
			std::string str = ss.str();
			Microsoft::VisualStudio::CppUnitTestFramework::Logger::WriteMessage(str.c_str());
		}

		void CheckRange(float start, float stop, double delta)
		{
			double maxErrorS = 0.0;
			double maxErrorC = 0.0;

			double maxErrorSInput = 0.0;
			double maxErrorCInput = 0.0;

			{

				float sf = 0.0f;
				float cf = 0.0f;
				fresnelf(1.72f, cf, sf);
			}

			for (double i = start; i < stop; i += delta)
			{
				double inputd = i;
				double sd = 0.0;
				double cd = 0.0;

				float inputf = static_cast<float>(inputd);
				float sf = 0.0f;
				float cf = 0.0f;

				fresnel(inputd, cd, sd);
				fresnelf(inputf, cf, sf);

				double sdelta = fabs(sd - sf) / sd;
				double cdelta = fabs(cd - cf) / cd;

				if (maxErrorS < sdelta)
				{
					maxErrorS = sdelta;
					maxErrorSInput = inputd;
				}
				if (maxErrorC < cdelta)
				{
					maxErrorC = cdelta;
					maxErrorCInput = inputd;
				}

			}
			std::stringstream ss("");

			ss << "Checking Range: " << start << " to " << stop << "\n" << std::fixed << std::setprecision(9);
			ss << "S error: " << (maxErrorS * 1000) << "%    Max S Error at: " << maxErrorSInput << "\n";
			ss << "C error: " << (maxErrorC * 1000) << "%    Max C Error at: " << maxErrorCInput << "\n";
			std::string str = ss.str();
			Microsoft::VisualStudio::CppUnitTestFramework::Logger::WriteMessage(str.c_str());
			Assert::IsTrue(maxErrorS < 0.001, L"Fresnel S has too much error.");
			Assert::IsTrue(maxErrorC < 0.001, L"Fresnel C has too much error.");

		}
		double FindThreshold(double startValue, double endValue, double delta, int iterations)
		{
			double thresholdValue = 0.0;
			double sErr = 0;
			double cErr = 0;
			bool thresholdReached = false;
			bool atLeastOnePass = false;
			constexpr double threshold = 0.00001;
			for (double i = startValue; i <= endValue; i += delta)
			{
				GetError(i, sErr, cErr, iterations + 0.001);


				if (sErr >= threshold || cErr >= threshold)
				{
					thresholdReached = true;
					break;
				}
				else
				{
					atLeastOnePass = true;
				}
				thresholdValue = i;
			}
			return thresholdValue;
		}


		double FindThreshold(float testValue, int startIterations, int endIterations, std::vector<float>& errorResults)
		{
			double thresholdValue = 0.0;
			double sErr = 0;
			double cErr = 0;
			bool thresholdReached = false;
			bool atLeastOnePass = false;
			for (int i = startIterations; i <= endIterations; ++i)
			{
				GetError(testValue, sErr, cErr, i + 0.001);

				errorResults[i - startIterations] = static_cast<float>(fmax(sErr, cErr));
				if (sErr >= 0.00001 || cErr >= 0.00001)
				{
					thresholdReached = true;
					//break;
				}
				else
				{
					atLeastOnePass = true;
				}
				thresholdValue = i;
			}
			return thresholdValue;
		}
		double FindThresholdValue(double startValue, double endValue, double delta, int iterations, std::vector<float>& errorResults)
		{
			double thresholdValue = 0.0;
			double sErr = 0;
			double cErr = 0;
			bool thresholdReached = false;
			bool atLeastOnePass = false;
			for (double i = startValue; i <= endValue; i += delta)
			{
				GetError(i, sErr, cErr, iterations + 0.001);

				const size_t errorIndex = static_cast<size_t>(((i - startValue) / delta) + 0.5);
				errorResults[errorIndex] = static_cast<float>(fmax(sErr, cErr));
				if (sErr >= 0.0001 || cErr >= 0.0001)
				{
					thresholdReached = true;
					//break;
				}
				else
				{
					atLeastOnePass = true;
				}
				thresholdValue = i;
			}
			return thresholdValue;
		}
		TEST_METHOD(MapThresholds)
		{
			std::stringstream ss("");
			int start = 20;
			int end = 50;
			float testVal = 1.8f;

			std::vector<float> errors = std::vector<float>(end-start+1);
			FindThreshold(testVal, start, end, errors);
			for (int i = start; i <= end; i++)
			{
				ss << i << "\t" << 100*errors[i - start] << "\n";
			}
			ss << "\n" << errors.size();
			std::string str = ss.str();
			Microsoft::VisualStudio::CppUnitTestFramework::Logger::WriteMessage(str.c_str());
		}
		TEST_METHOD(MapThresholds3)
		{
			std::stringstream ss("");
			int start = 10;
			int end = 50;
			double testStart = 1.8;
			double testEnd = 2.7;
			double delta = 0.000005;

			ss << std::fixed << std::setprecision(9);
			for (int i = start; i <= end; i++)
			{
				double val = FindThreshold(testStart, testEnd, delta, i);

				ss << i << " : " <<val << "\n";
			}
			std::string str = ss.str();
			Microsoft::VisualStudio::CppUnitTestFramework::Logger::WriteMessage(str.c_str());
		}
		TEST_METHOD(MapThresholds2)
		{
			std::stringstream ss("");
			double startVal = 2.5;
			double endVal = 2.7;
			double delta = 0.000001;
			int iterations = 38;

			std::vector<float> errors = std::vector<float>(static_cast<size_t>(((endVal - startVal) / delta) + 2.5));
			FindThresholdValue(startVal, endVal, delta, iterations, errors);
			float maxErrorVal = 0;
			float maxError = 0;
			for (size_t i = 0; i < errors.size(); i++)
			{
				if (i>0 && (i % 1000 == 0))
				{
					ss << maxErrorVal << "\t" << 100 * maxError;
					if (maxError >= 0.000001)
					{
						ss << "\t#####";
					}

					ss << "\n";
					maxError = 0;
				}

				if (errors[i] > maxError)
				{
					maxError = errors[i];
					maxErrorVal = static_cast<float>(startVal + (static_cast<double>(i) * delta));
				}
			}
			ss << "\n" << errors.size();
			std::string str = ss.str();
			Microsoft::VisualStudio::CppUnitTestFramework::Logger::WriteMessage(str.c_str());
		}

		TEST_METHOD(CheckRanges)
		{
			CheckRange(0.0f, MazeMap::FIRST_BOUNDARY, 0.00001);
			CheckRange(MazeMap::FIRST_BOUNDARY, MazeMap::SECOND_BOUNDARY, 0.000001);
			CheckRange(MazeMap::SECOND_BOUNDARY, 15.0f, 0.00001f);
		}

	};
}
