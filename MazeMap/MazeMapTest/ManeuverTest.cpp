#include "pch.h"
#include "Templates.h"
#include "CppUnitTest.h"
#include "MazeRef.h"
#include "..\MazeMap\Maneuver.h"
#include "..\MazeMap\ManeuverSet.h"
#include "../MazeMap/ManeuverPath.h"
#include "../MazeMap/ManeuverQueue.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

std::wstring CodeString(MazeMap::ManeuverCode code)
{
	if (code != MazeMap::MC_NONE && code <= MazeMap::S31)
	{
		std::wstringstream ss(L"S");
		ss << (static_cast<uint16_t>(code));
		return std::wstring(ss.str());
	}

	switch (code)
	{
	case MazeMap::MC_NONE: return std::wstring(L"MC_NONE");
	case MazeMap::IP45: return std::wstring(L"IP45");
	case MazeMap::IP90: return std::wstring(L"IP90");
	case MazeMap::IP135: return std::wstring(L"IP135");
	case MazeMap::IP180: return std::wstring(L"IP180");
	case MazeMap::S45SS: return std::wstring(L"S45SS");
	case MazeMap::S45SD: return std::wstring(L"S45SD");
	case MazeMap::S45LS: return std::wstring(L"S45LS");
	case MazeMap::S45LD: return std::wstring(L"S45LD");
	case MazeMap::S90SS: return std::wstring(L"S90SS");
	case MazeMap::S90LS: return std::wstring(L"S90LS");
	case MazeMap::S90SD: return std::wstring(L"S90SD");
	case MazeMap::S90LD: return std::wstring(L"S90LD");
	case MazeMap::S135SS: return std::wstring(L"S135SS");
	case MazeMap::S135LS: return std::wstring(L"S135LS");
	case MazeMap::S135SD: return std::wstring(L"S135SD");
	case MazeMap::S135LD: return std::wstring(L"S135LD");
	case MazeMap::S180SS: return std::wstring(L"S180SS");
	case MazeMap::S180LS: return std::wstring(L"S180LS");
	case MazeMap::S90ELD: return std::wstring(L"S90ELD");
	case MazeMap::S180ELS: return std::wstring(L"S180ELS");
	case MazeMap::IP45_M: return std::wstring(L"IP45_M");
	case MazeMap::IP90_M: return std::wstring(L"IP90_M");
	case MazeMap::IP135_M: return std::wstring(L"IP135_M");
	case MazeMap::IP180_M: return std::wstring(L"IP180_M");
	case MazeMap::S45SS_M: return std::wstring(L"S45SS_M");
	case MazeMap::S45SD_M: return std::wstring(L"S45SD_M");
	case MazeMap::S45LS_M: return std::wstring(L"S45LS_M");
	case MazeMap::S45LD_M: return std::wstring(L"S45LD_M");
	case MazeMap::S90SS_M: return std::wstring(L"S90SS_M");
	case MazeMap::S90LS_M: return std::wstring(L"S90LS_M");
	case MazeMap::S90SD_M: return std::wstring(L"S90SD_M");
	case MazeMap::S90LD_M: return std::wstring(L"S90LD_M");
	case MazeMap::S135SS_M: return std::wstring(L"S135SS_M");
	case MazeMap::S135LS_M: return std::wstring(L"S135LS_M");
	case MazeMap::S135SD_M: return std::wstring(L"S135SD_M");
	case MazeMap::S135LD_M: return std::wstring(L"S135LD_M");
	case MazeMap::S180SS_M: return std::wstring(L"S180SS_M");
	case MazeMap::S180LS_M: return std::wstring(L"S180LS_M");
	case MazeMap::S90ELD_M: return std::wstring(L"S90ELD_M");
	case MazeMap::S180ELS_M: return std::wstring(L"S180ELS_M");
	}

	return std::wstring(L"UNKNOWN");
}
namespace MazeMap
{
	TEST_CLASS(ManeuverTest)
	{
	public:
		ManeuverSet& ms = ManeuverSet::GetSet();

		static Vehicle MakeTestVehicle(
			float maxForwardAcceleration,
			float maxLateralAcceleration,
			float maxAngularAcceleration,
			float maxRotationalVelocity,
			float maxSpeed)
		{
			Vehicle vehicle;
			vehicle.SetMaxForwardAcceleration(maxForwardAcceleration);
			vehicle.SetMaxLateralAcceleration(maxLateralAcceleration);
			vehicle.SetMaxAngularAcceleration(maxAngularAcceleration);
			vehicle.SetMaxRotationalVelocity(maxRotationalVelocity);
			vehicle.SetMaxSpeed(maxSpeed);
			return vehicle;
		}

		void ReverseTest(const Maneuver& man)
		{
			DirectionalLocation dirLoc(MazeLocation(15, 15), Direction::Up);
			if (!man.SupportsStraightEntry())
			{
				dirLoc = dirLoc.Turn(Right45);
			}
			DirectionalLocation result = dirLoc;
			result = man.Move(result,false);
			ManeuverCode code = man.GetBackwardsManeuverID();
			auto& reverseManeuver = ms[code];

			DirectionalLocation tmp = result.Turn(Reverse);

			result = reverseManeuver.Move(tmp, code & ManeuverCode::MIRRORED_MANEUVER_FLAG).Turn(Reverse);
			std::wstring message = CodeString(man.GetManeuverID());
			Assert::AreEqual(dirLoc, result, message.c_str());
		}


		float StraightDistanceMeters(ManeuverCode code)
		{
			return static_cast<float>(static_cast<uint8_t>(code)) * 0.5f * Maze::GetCellDimension() / 100.0f;
		}

		float ReachableStraightSpeed(const Vehicle& vehicle, float entrySpeed, ManeuverCode code)
		{
			float distance = StraightDistanceMeters(code);
			return sqrtf((entrySpeed * entrySpeed) + (2.0f * vehicle.GetMaxForwardAcceleration() * distance));
		}

		void BuildManeuverInstances(DirectionalLocation start, const ManeuverCode* codes, uint16_t count, ManeuverInstance* result)
		{
			DirectionalLocation current = start;
			ManeuverSet& set = ManeuverSet::GetSet();
			for (uint16_t i = 0; i < count; ++i)
			{
				result[i] = ManeuverInstance(codes[i], current);
				current = set.Move(codes[i], current);
			}
		}
		TEST_METHOD(ReverseTIP45)
		{
			auto man = TurnInPlace45();
			ReverseTest(man);

		}
		TEST_METHOD(ReverseTIP90)
		{
			auto man=TurnInPlace90();
			ReverseTest(man);

		}
		TEST_METHOD(ReverseTIP135)
		{
			auto man = TurnInPlace135();
			ReverseTest(man);

		}
		TEST_METHOD(ReverseTIP180)
		{
			auto man = TurnInPlace180();
			ReverseTest(man);

		}
		TEST_METHOD(ReverseSmooth45ShortStraight)
		{
			auto man = Smooth45ShortStraight();
			ReverseTest(man);

		}
		TEST_METHOD(ReverseSmooth45ShortDiagonal)
		{
			auto man = Smooth45ShortDiagonal();
			ReverseTest(man);

		}
		TEST_METHOD(ReverseSmooth45LongDiagonal)
		{
			auto man = Smooth45LongDiagonal();
			ReverseTest(man);

		}
		TEST_METHOD(ReverseSmooth45LongStraight)
		{
			auto man = Smooth45LongStraight();
			ReverseTest(man);

		}
		/*
		S45S  = 36,
		S45LS = 37,
		S45LD = 38,
		S90SS = 39,
		S90D  = 40,
		S90LS = 41,
		S135S = 42,
		S135D = 43,*/
		TEST_METHOD(ReverseSmooth90ShortStraight)
		{
			auto man = Smooth90ShortStraight();
			ReverseTest(man);

		}
		TEST_METHOD(ReverseSmooth90Diagonal)
		{
			auto man = Smooth90ShortDiagonal();
			ReverseTest(man);

		}
		TEST_METHOD(ReverseSmooth90LongStraight)
		{
			auto man = Smooth90LongStraight();
			ReverseTest(man);

		}
		TEST_METHOD(ReverseSmooth135Straight)
		{
			auto man = Smooth135LongStraight();
			ReverseTest(man);

		}
		TEST_METHOD(ReverseSmooth135ShortStraight)
		{
			auto man = Smooth135ShortStraight();
			ReverseTest(man);

		}
		TEST_METHOD(ReverseSmooth135Diagonal)
		{
			auto man = Smooth135LongDiagonal();
			ReverseTest(man);

		}
		TEST_METHOD(ReverseSmooth135ShortDiagonal)
		{
			auto man = Smooth135ShortDiagonal();
			ReverseTest(man);

		}
		TEST_METHOD(Smooth45LongStraightForward1)
		{
			auto man = Smooth45LongStraight();

			DirectionalLocation dirLoc(MazeLocation(15, 15), Direction::Up);
			DirectionalLocation expected(MazeLocation(16, 18), Direction::UpRight);
			dirLoc = man.Move(dirLoc, false);
			Assert::IsTrue(dirLoc == expected);

		}
		TEST_METHOD(Smooth45LongStraightForward2)
		{
			auto man = Smooth45LongStraight();

			DirectionalLocation dirLoc(MazeLocation(15, 15), Direction::Down);
			DirectionalLocation expected(MazeLocation(14, 12), Direction::DownLeft);
			dirLoc = man.Move(dirLoc, false);
			Assert::IsTrue(dirLoc == expected);

		}
		TEST_METHOD(Smooth45LongStraightForward3)
		{
			auto man = Smooth45LongStraight();

			DirectionalLocation dirLoc(MazeLocation(15, 15), Direction::Left);
			DirectionalLocation expected(MazeLocation(12, 16), Direction::UpLeft);
			dirLoc = man.Move(dirLoc, false);
			Assert::AreEqual( expected, dirLoc);

		}
		TEST_METHOD(Smooth45LongStraightMirrored)
		{
			auto man = Smooth45LongStraight();

			DirectionalLocation dirLoc(MazeLocation(15, 15), Direction::Up);
			DirectionalLocation expected(MazeLocation(14, 18), Direction::UpLeft);
			dirLoc = man.Move(dirLoc, true);
			Assert::IsTrue(dirLoc == expected);

		}
		TEST_METHOD(Smooth45LongStraightMirrored2)
		{
			auto man = Smooth45LongStraight();

			DirectionalLocation dirLoc(MazeLocation(15, 15), Direction::Down);
			DirectionalLocation expected(MazeLocation(16, 12), Direction::DownRight);
			dirLoc = man.Move(dirLoc, true);
			Assert::IsTrue(dirLoc == expected);

		}
		TEST_METHOD(Smooth45LongStraightMirrored3)
		{
			auto man = Smooth45LongStraight();

			DirectionalLocation dirLoc(MazeLocation(18, 16), Direction::Left);
			DirectionalLocation expected(MazeLocation(15, 15), Direction::DownLeft);
			dirLoc = man.Move(dirLoc, true);
			Assert::AreEqual(expected, dirLoc);

		}
		TEST_METHOD(Smooth45LongDiagonalForward)
		{
			auto man = Smooth45LongDiagonal();

			DirectionalLocation dirLoc(MazeLocation(15, 15), Direction::UpLeft);
			DirectionalLocation expected(MazeLocation(14, 18), Direction::Up);
			dirLoc = man.Move(dirLoc, false);
			Assert::AreEqual(expected, dirLoc);

		}
		TEST_METHOD(Smooth45LongDiagonalForward2)
		{
			auto man = Smooth45LongDiagonal();

			DirectionalLocation dirLoc(MazeLocation(15, 15), Direction::UpRight);
			DirectionalLocation expected(MazeLocation(18, 16), Direction::Right);
			dirLoc = man.Move(dirLoc, false);
			Assert::AreEqual(expected, dirLoc);

		}
		TEST_METHOD(Smooth45LongDiagonalForward3)
		{
			auto man = Smooth45LongDiagonal();

			DirectionalLocation dirLoc(MazeLocation(15, 15), Direction::DownLeft);
			DirectionalLocation expected(MazeLocation(12, 14), Direction::Left);
			dirLoc = man.Move(dirLoc, false);
			Assert::AreEqual(expected, dirLoc);

		}
		TEST_METHOD(Smooth45LongDiagonalMirrored)
		{
			auto man = Smooth45LongDiagonal();

			DirectionalLocation dirLoc(MazeLocation(15, 15), Direction::UpRight);
			DirectionalLocation expected(MazeLocation(16, 18), Direction::Up);
			dirLoc = man.Move(dirLoc, true);
			Assert::AreEqual(expected, dirLoc);

		}
		TEST_METHOD(CostTest1)
		{
			Vehicle v = MakeTestVehicle(15.0f, 18.0f, 35.0f, 4.5f, 5000.0f);
			float cellDim = 0.18f;
			auto man1 = Smooth90LongStraight();
			auto man2 = Smooth90ShortStraight();
			auto man3 = TurnInPlace45();
			float cost1 = man1.GetCost(v);
			float cost2 = v.GetStraightLineCost(29 * cellDim, 0.0f, man1.GetEntrySpeed(v));
			float totalCost = cost1 + cost2;
			float cost3 = man2.GetCost(v);
			return;
		}
		TEST_METHOD(ValidTest1)
		{
			Vehicle v = MakeTestVehicle(15.0f, 18.0f, 35.0f, 4.5f, 5000.0f);
			float cellDim = 0.18f;
			Mazes::SetupMazes();
			Maze& m = Mazes::GetSingleTurnMaze();

			DirectionalLocation dirLoc(3, 31, Left);
			auto man1 = Smooth90LongStraight();
			bool result = man1.IsValidMove(dirLoc, m, true);
			Assert::IsTrue(result);
			return;
		}
		TEST_METHOD(SpeedTest1)
		{
			Vehicle v = MakeTestVehicle(15.0f, 18.0f, 35.0f, 4.5f, 5000.0f);
			float cellDim = 0.18f;
			ManeuverPath p1 = ManeuverPath();
			p1.push_back(S29);
			p1.push_back(S90SS);
			p1.push_back(S1);
			ManeuverPath p2 = ManeuverPath();
			p2.push_back(S28);
			p2.push_back(S90LS);
			ManeuverSet& ms = ManeuverSet::GetSet();
			float entry1 = ms.GetEntrySpeed(S90SS, v, cellDim);
			float entry2 = ms.GetEntrySpeed(S90LS, v, cellDim);

			float cc1 = ms.GetCost(S90SS, v, cellDim);
			float cc2 = ms.GetCost(S90LS, v, cellDim);

			float s1 = v.GetStraightLineCost(29 * cellDim * 0.5f, 0, entry1);
			float s2 = v.GetStraightLineCost(28 * cellDim * 0.5f, 0, entry2);

			float c1 = p1.Cost(v, cellDim);
			float c2 = p2.Cost(v, cellDim);
			return;
		}
		TEST_METHOD(ToHalfStepPathTest)
		{
			Vehicle v = MakeTestVehicle(15.0f, 18.0f, 35.0f, 4.5f, 5000.0f);
			float cellDim = 0.18f;

			DirectionalLocation start(1, 1, Up);

			ManeuverPath p1 = ManeuverPath();
			p1.push_back(S28);
			p1.push_back(S90LS);
			/*p1.push_back(IP90);
			p1.push_back(IP135);
			p1.push_back(IP180);

			// (15,15,Right)
			p1.push_back(S45SS);
			p1.push_back(S45SD);
			p1.push_back(S45SS);
			p1.push_back(S45SD);

			p1.push_back(S90SS);
			p1.push_back(S90LS);
			p1.push_back(S90SD);
			p1.push_back(S90LD);

			p1.push_back(S135SS);
			p1.push_back(S135LS);
			p1.push_back(S135SD);
			p1.push_back(S135LD);*/

			HalfStepPath<PATH_SIZE * 2> p2 = HalfStepPath<PATH_SIZE * 2>();
			p2.push_back(MazeLocation(1, 1));
			p2.push_back(MazeLocation(1, 2));
			p2.push_back(MazeLocation(1, 3));
			p2.push_back(MazeLocation(1, 4));
			p2.push_back(MazeLocation(1, 5));
			p2.push_back(MazeLocation(1, 6));
			p2.push_back(MazeLocation(1, 7));
			p2.push_back(MazeLocation(1, 8));
			p2.push_back(MazeLocation(1, 9));
			p2.push_back(MazeLocation(1, 10));
			p2.push_back(MazeLocation(1, 11));
			p2.push_back(MazeLocation(1, 12));
			p2.push_back(MazeLocation(1, 13));
			p2.push_back(MazeLocation(1, 14));
			p2.push_back(MazeLocation(1, 15));
			p2.push_back(MazeLocation(1, 16));
			p2.push_back(MazeLocation(1, 17));
			p2.push_back(MazeLocation(1, 18));
			p2.push_back(MazeLocation(1, 19));
			p2.push_back(MazeLocation(1, 20));
			p2.push_back(MazeLocation(1, 21));
			p2.push_back(MazeLocation(1, 22));
			p2.push_back(MazeLocation(1, 23));
			p2.push_back(MazeLocation(1, 24));
			p2.push_back(MazeLocation(1, 25));
			p2.push_back(MazeLocation(1, 26));
			p2.push_back(MazeLocation(1, 27));
			p2.push_back(MazeLocation(1, 28));
			p2.push_back(MazeLocation(1, 29));
			p2.push_back(MazeLocation(1, 30));
			p2.push_back(MazeLocation(2, 31));
			p2.push_back(MazeLocation(3, 31));
			HalfStepPath<PATH_SIZE * 2> p3 = HalfStepPath<PATH_SIZE * 2>();
			p1.ToHalfStepPath(start, p3);
			/*
		S45SS = 36,
		S45SD = 37,
		S45LS = 38,
		S45LD = 39,
		S90SS = 40,
		S90LS = 41,
		S90SD = 42,
		S90LD = 43,
		S135SS = 44,
		S135LS = 45,
		S135SD = 46,
		S135LD = 47,*/
			return;
		}

		/*
		S45S  = 36,
		S45LS = 37,
		S45LD = 38,
		S90SS = 39,
		S90D  = 40,
		S90LS = 41,
		S135S = 42,
		S135D = 43,*/
		TEST_METHOD(ManeuverQueueBuildFromPath)
		{
			DirectionalLocation start(MazeLocation(15, 15), Up);
			ManeuverPath path = ManeuverPath();
			path.push_back(S2);
			path.push_back(S90SS);
			path.push_back(S1);

			ManeuverQueue queue(path, start);
			ManeuverSet& set = ManeuverSet::GetSet();

			Assert::AreEqual(static_cast<uint16_t>(3), queue.size());
			Assert::AreEqual(start, queue[0].GetStart());
			Assert::AreEqual(set.Move(S2, start), queue[1].GetStart());
			Assert::AreEqual(set.Move(S90SS, queue[1].GetStart()), queue[2].GetStart());
		}

		TEST_METHOD(ManeuverQueueAppendsSequentialCommands)
		{
			DirectionalLocation start(MazeLocation(15, 15), Up);
			ManeuverQueue queue = ManeuverQueue();

			Assert::IsTrue(queue.push_back(S4, start));
			DirectionalLocation secondStart = queue[0].GetEnd();
			Assert::IsTrue(queue.push_back(S90SS));
			Assert::AreEqual(secondStart, queue[1].GetStart());
		}

		TEST_METHOD(ManeuverQueueHonorsCapacity)
		{
			DirectionalLocation start(MazeLocation(100, 100), Up);
			ManeuverQueue queue = ManeuverQueue();

			Assert::IsTrue(queue.push_back(S1, start));
			for (uint16_t i = 1; i < MANEUVER_QUEUE_CAPACITY; ++i)
			{
				Assert::IsTrue(queue.push_back(S1));
			}

			Assert::AreEqual(MANEUVER_QUEUE_CAPACITY, queue.size());
			Assert::IsFalse(queue.push_back(S1));
		}

		TEST_METHOD(ManeuverQueueComputesStraightTurnStraightSpeeds)
		{
			const float tolerance = 0.0001f;
			Vehicle v = MakeTestVehicle(2.0f, 4.0f, 35.0f, 5.0f, 5000.0f);
			ManeuverQueue queue = ManeuverQueue();

			Assert::IsTrue(queue.push_back(S10, DirectionalLocation(MazeLocation(15, 15), Up)));
			Assert::IsTrue(queue.push_back(S90SS));
			Assert::IsTrue(queue.push_back(S10));
			queue.ComputeSpeeds(v, 0.0f, 0.0f);

			float turnSpeed = ManeuverSet::GetSet()[S90SS].GetVMax(v);
			Assert::AreEqual(0.0f, queue[0].GetEntrySpeed(), tolerance);
			Assert::AreEqual(turnSpeed, queue[0].GetExitSpeed(), tolerance);
			Assert::AreEqual(turnSpeed, queue[1].GetEntrySpeed(), tolerance);
			Assert::AreEqual(turnSpeed, queue[1].GetExitSpeed(), tolerance);
			Assert::AreEqual(turnSpeed, queue[2].GetEntrySpeed(), tolerance);
			Assert::AreEqual(0.0f, queue[2].GetExitSpeed(), tolerance);
		}

		TEST_METHOD(ManeuverQueueFreshSpeedsUpdatePriorStraight)
		{
			const float tolerance = 0.0001f;
			Vehicle v = MakeTestVehicle(2.0f, 4.0f, 35.0f, 5.0f, 5000.0f);
			ManeuverQueue queue = ManeuverQueue();

			Assert::IsTrue(queue.push_back(S10, DirectionalLocation(MazeLocation(15, 15), Up)));
			queue.ComputeSpeeds(v, 0.0f, 0.0f);
			Assert::AreEqual(0.0f, queue[0].GetExitSpeed(), tolerance);

			Assert::IsTrue(queue.push_back(S90SS));
			queue.ComputeFreshSpeeds(v, 1);

			float turnSpeed = ManeuverSet::GetSet()[S90SS].GetVMax(v);
			Assert::AreEqual(turnSpeed, queue[0].GetExitSpeed(), tolerance);
			Assert::AreEqual(turnSpeed, queue[1].GetEntrySpeed(), tolerance);
			Assert::AreEqual(turnSpeed, queue[1].GetExitSpeed(), tolerance);
		}

		TEST_METHOD(ManeuverQueueComputesConstantSpeedAcrossAdjacentTurns)
		{
			const float tolerance = 0.0001f;
			Vehicle v = MakeTestVehicle(2.0f, 4.0f, 35.0f, 5.0f, 5000.0f);
			DirectionalLocation start(MazeLocation(15, 15), Up);
			ManeuverInstance fresh[2] =
			{
				ManeuverInstance(S90LS, start),
				ManeuverInstance(S45SS, ManeuverSet::GetSet().Move(S90LS, start))
			};

			ManeuverQueue::ComputeSpeeds(v, fresh, 2, v.GetMaxSpeed(), v.GetMaxSpeed());

			float expected = ManeuverSet::GetSet()[S90LS].GetVMax(v);
			float secondLimit = ManeuverSet::GetSet()[S45SS].GetVMax(v);
			if (secondLimit < expected)
			{
				expected = secondLimit;
			}

			Assert::AreEqual(expected, fresh[0].GetEntrySpeed(), tolerance);
			Assert::AreEqual(expected, fresh[0].GetExitSpeed(), tolerance);
			Assert::AreEqual(expected, fresh[1].GetEntrySpeed(), tolerance);
			Assert::AreEqual(expected, fresh[1].GetExitSpeed(), tolerance);
		}
		TEST_METHOD(ManeuverQueueSpeedPass_ConsecutiveTurnsKeepSameSpeed)
		{
			const float tolerance = 0.0001f;
			Vehicle v = MakeTestVehicle(3.0f, 4.0f, 35.0f, 5.0f, 5000.0f);
			ManeuverCode codes[3] = { S90LS, S90SS, S180SS };
			ManeuverInstance fresh[3];
			BuildManeuverInstances(DirectionalLocation(MazeLocation(15, 15), Up), codes, 3, fresh);

			ManeuverQueue::ComputeSpeeds(v, fresh, 3, v.GetMaxSpeed(), v.GetMaxSpeed());

			float sharedSpeed = fresh[0].GetEntrySpeed();
			for (uint16_t i = 0; i < 3; ++i)
			{
				Assert::AreEqual(sharedSpeed, fresh[i].GetEntrySpeed(), tolerance);
				Assert::AreEqual(sharedSpeed, fresh[i].GetExitSpeed(), tolerance);
			}
		}

		TEST_METHOD(ManeuverQueueSpeedPass_ConsecutiveTurnsUseTightestLimit)
		{
			const float tolerance = 0.0001f;
			Vehicle v = MakeTestVehicle(3.0f, 4.0f, 35.0f, 5.0f, 5000.0f);
			ManeuverCode codes[3] = { S90LS, S90SS, S180SS };
			ManeuverInstance fresh[3];
			BuildManeuverInstances(DirectionalLocation(MazeLocation(15, 15), Up), codes, 3, fresh);

			ManeuverQueue::ComputeSpeeds(v, fresh, 3, v.GetMaxSpeed(), v.GetMaxSpeed());

			float expected = ManeuverSet::GetSet()[S90LS].GetVMax(v);
			float currentLimit = ManeuverSet::GetSet()[S90SS].GetVMax(v);
			if (currentLimit < expected)
			{
				expected = currentLimit;
			}
			currentLimit = ManeuverSet::GetSet()[S180SS].GetVMax(v);
			if (currentLimit < expected)
			{
				expected = currentLimit;
			}

			for (uint16_t i = 0; i < 3; ++i)
			{
				Assert::AreEqual(expected, fresh[i].GetEntrySpeed(), tolerance);
				Assert::AreEqual(expected, fresh[i].GetExitSpeed(), tolerance);
			}
		}

		TEST_METHOD(ManeuverQueueSpeedPass_TurnStraightTurnAccelerationLimitsSecondTurn)
		{
			const float tolerance = 0.0001f;
			Vehicle v = MakeTestVehicle(1.0f, 4.0f, 35.0f, 5.0f, 5000.0f);
			ManeuverCode codes[3] = { S90SS, S1, S90LS };
			ManeuverInstance fresh[3];
			BuildManeuverInstances(DirectionalLocation(MazeLocation(15, 15), Up), codes, 3, fresh);

			ManeuverQueue::ComputeSpeeds(v, fresh, 3, v.GetMaxSpeed(), v.GetMaxSpeed());

			float firstTurnSpeed = ManeuverSet::GetSet()[S90SS].GetVMax(v);
			float secondTurnLimit = ManeuverSet::GetSet()[S90LS].GetVMax(v);
			float expectedSecondTurnSpeed = ReachableStraightSpeed(v, firstTurnSpeed, S1);
			if (secondTurnLimit < expectedSecondTurnSpeed)
			{
				expectedSecondTurnSpeed = secondTurnLimit;
			}

			Assert::AreEqual(firstTurnSpeed, fresh[0].GetEntrySpeed(), tolerance);
			Assert::AreEqual(firstTurnSpeed, fresh[0].GetExitSpeed(), tolerance);
			Assert::AreEqual(firstTurnSpeed, fresh[1].GetEntrySpeed(), tolerance);
			Assert::AreEqual(expectedSecondTurnSpeed, fresh[1].GetExitSpeed(), tolerance);
			Assert::AreEqual(expectedSecondTurnSpeed, fresh[2].GetEntrySpeed(), tolerance);
			Assert::AreEqual(expectedSecondTurnSpeed, fresh[2].GetExitSpeed(), tolerance);
			Assert::IsTrue(expectedSecondTurnSpeed < secondTurnLimit);
		}

		TEST_METHOD(ManeuverQueueSpeedPass_StraightEntryAndExitFollowAdjacentTurnLimits)
		{
			const float tolerance = 0.0001f;
			Vehicle v = MakeTestVehicle(2.0f, 4.0f, 35.0f, 5.0f, 5000.0f);
			ManeuverCode codes[3] = { S90LS, S6, S90SS };
			ManeuverInstance fresh[3];
			BuildManeuverInstances(DirectionalLocation(MazeLocation(15, 15), Up), codes, 3, fresh);

			ManeuverQueue::ComputeSpeeds(v, fresh, 3, v.GetMaxSpeed(), v.GetMaxSpeed());

			float firstTurnLimit = ManeuverSet::GetSet()[S90LS].GetVMax(v);
			float secondTurnLimit = ManeuverSet::GetSet()[S90SS].GetVMax(v);
			Assert::AreEqual(firstTurnLimit, fresh[0].GetEntrySpeed(), tolerance);
			Assert::AreEqual(firstTurnLimit, fresh[0].GetExitSpeed(), tolerance);
			Assert::AreEqual(firstTurnLimit, fresh[1].GetEntrySpeed(), tolerance);
			Assert::AreEqual(secondTurnLimit, fresh[1].GetExitSpeed(), tolerance);
			Assert::AreEqual(secondTurnLimit, fresh[2].GetEntrySpeed(), tolerance);
			Assert::AreEqual(secondTurnLimit, fresh[2].GetExitSpeed(), tolerance);
		}
		TEST_METHOD(MovementTest)
		{
			const ManeuverSet& ms = ManeuverSet::GetSet();

			for (uint8_t i = 0; i < ms.size(); i++)
			{
				const Maneuver& man = ms[i];

				DirectionalLocation loc(15, 15, Up);
				DirectionalLocation result = loc;
				DirectionalLocation expected = man.Move(loc, false);

				for (uint8_t j = 0; j < man.GetStepCount(); ++j)
				{
					RelativeDirectionalDistance rdd = man.GetStep(j);
					result = result.Turn(rdd.GetDirection());
					for (uint8_t k = 0; k < rdd.GetDistance(); k++)
					{
						result = result.MoveForward(1);
					}
				}
				Assert::AreEqual(expected, result);
			}
		};
		TEST_METHOD(MovementReverseTest)
		{
			const ManeuverSet& ms = ManeuverSet::GetSet();

			for (uint8_t i = 0; i < ms.size(); i++)
			{
				const Maneuver& man = ms[i];
				ReverseTest(man);
			}
		};
		TEST_METHOD(MovementMirroredTest)
		{
			const ManeuverSet& ms = ManeuverSet::GetSet();

			for (uint8_t i = 0; i < ms.size(); i++)
			{
				const Maneuver& man = ms[i];

				DirectionalLocation loc(15, 15, Up);
				DirectionalLocation result = loc;
				DirectionalLocation expected = man.Move(loc, true);

				for (uint8_t j = 0; j < man.GetStepCount(); ++j)
				{
					RelativeDirectionalDistance rdd = man.GetStep(j);
					result = result.Turn(-rdd.GetDirection());
					for (uint8_t k = 0; k < rdd.GetDistance(); k++)
					{
						result = result.MoveForward(1);
					}
				}
				Assert::AreEqual(expected, result);
			}
		};
		TEST_METHOD(MovementMirroredReverseTest)
		{
			const ManeuverSet& ms = ManeuverSet::GetSet();

			for (uint8_t i = 0; i < ms.size(); i++)
			{
				const Maneuver& man = ms[i];
				const Maneuver& man2 = ms[man.GetBackwardsManeuverID()];

				DirectionalLocation loc(15, 15, Up);
				DirectionalLocation result = ms.Move(man.GetBackwardsManeuverID() ^ MIRRORED_MANEUVER_FLAG, man.Move(loc, true).Turn(Reverse)).Turn(Reverse);
				DirectionalLocation expected = loc;

				Assert::AreEqual(expected, result);
			}
		};
		TEST_METHOD(IsValidTest)
		{
			Maze& m = Mazes::GetSingleTurnMaze();
			m.PreCalculate();
			MazeLocation origin = MazeLocation::CellCenter((m.GetGoalLowerLeft() >> Up));
			Direction facing = Left;

			Maneuver& man = Smooth90LongStraight();
			bool result = man.IsValidMove(DirectionalLocation(origin, facing), m, true);

			Assert::IsTrue(result);
		};
	};

}






