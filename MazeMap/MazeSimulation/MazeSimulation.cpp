// MazeSimulation.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
//#define _WINDOWS
#define MAZE_EXPORT
#include <stdint.h>
#include <windows.h>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include "..\MazeMap\Vehicle.h"
#include "..\MazeMap\Maze.h"
#include "..\MazeMap\PathFinder.h"
#include "..\MazeMap\DirectionalPathFinder.h"
#include "..\MazeMap\MouseUkf.h"
#include "Mazes.h"
#include "SimVehicle.h"
#include "../MazeMap/ManeuverPathFinder.h"

MazeMap::Vehicle vehicle = MazeMap::Vehicle();
MazeMap::Maze maze = MazeMap::Maze();
MazeMap::DirectionalPathFinder advancedPathFinder = MazeMap::DirectionalPathFinder(maze, vehicle);
MazeMap::ManeuverPathFinder superAdvancedPathFinder = MazeMap::ManeuverPathFinder(maze, vehicle);
SimVehicle simVehicle = SimVehicle();
MazeMap::Path<PATH_SIZE> p = MazeMap::Path<PATH_SIZE>();
constexpr int PROCESSING_CYCLES = 10000;

namespace
{
    constexpr const char* kOpenFloorUkfBenchmarkArg = "--benchmark-open-floor-ukf-stationary";
    constexpr uint32_t kDefaultOpenFloorUkfBenchmarkIterations = 200000U;
    constexpr uint32_t kOpenFloorUkfBenchmarkWarmupIterations = 2048U;
    constexpr float kOpenFloorUkfBenchmarkDtSeconds = 0.001f;
    constexpr float kOpenFloorUkfBenchmarkStationaryGyroRawRadps = 0.015f;

    MazeMap::VehicleState::StateMatrix BuildOpenFloorBenchmarkCovariance()
    {
        MazeMap::VehicleState::StateMatrix covariance =
            MazeMap::VehicleState::StateMatrix::Identity() * 1.0e-3f;
        covariance(MazeMap::VehicleState::kOmegaL, MazeMap::VehicleState::kOmegaL) = 0.25f;
        covariance(MazeMap::VehicleState::kOmegaR, MazeMap::VehicleState::kOmegaR) = 0.25f;
        covariance(MazeMap::VehicleState::kBgz, MazeMap::VehicleState::kBgz) = 0.01f;
        return covariance;
    }

    void ResetOpenFloorBenchmarkUkf(MazeMap::MouseUkfFacade& ukf)
    {
        MazeMap::VehicleState::StateVector state = MazeMap::VehicleState::StateVector::Zero();
        (void)ukf.reset(state, BuildOpenFloorBenchmarkCovariance());
    }

    bool ExecuteOpenFloorStationaryMeasurementCycle(
        MazeMap::MouseUkfFacade& ukf,
        float dtSeconds,
        const MazeMap::ControlInput& control,
        const MazeMap::EncoderObs& encoderObservation,
        const MazeMap::ImuAccelObs& accelObservation,
        float rawGyroRadps)
    {
        if (!ukf.predict(dtSeconds, control))
        {
            return false;
        }

        const MazeMap::MeasurementUpdateResult encoderUpdate = ukf.updateEncoderPair(encoderObservation, dtSeconds);
        if (!encoderUpdate.accepted)
        {
            return false;
        }

        const MazeMap::MeasurementUpdateResult yawUpdate = ukf.updateYawRate(rawGyroRadps);
        if (!yawUpdate.accepted)
        {
            return false;
        }

        const MazeMap::MeasurementUpdateResult accelUpdate = ukf.updatePlanarAccel(accelObservation);
        if (!accelUpdate.accepted)
        {
            return false;
        }

        return true;
    }

    uint32_t ParseBenchmarkIterations(const char* argument)
    {
        if (argument == nullptr || argument[0] == '\0')
        {
            return kDefaultOpenFloorUkfBenchmarkIterations;
        }

        char* end = nullptr;
        const unsigned long parsed = std::strtoul(argument, &end, 10);
        if (end == argument || *end != '\0' || parsed == 0UL)
        {
            return kDefaultOpenFloorUkfBenchmarkIterations;
        }

        return static_cast<uint32_t>(parsed);
    }

    bool IsOpenFloorUkfBenchmarkArg(const char* argument)
    {
        return argument != nullptr && std::strcmp(argument, kOpenFloorUkfBenchmarkArg) == 0;
    }

    int RunOpenFloorUkfStationaryBenchmark(uint32_t iterations)
    {
        MazeMap::MouseUkfFacade ukf;
        ResetOpenFloorBenchmarkUkf(ukf);

        const MazeMap::PlantParams& params = ukf.ukf().params();
        MazeMap::ControlInput control{};
        control.leftMotorCommand = 0.0f;
        control.rightMotorCommand = 0.0f;
        control.fanDutyCycle = 0.80f;
        control.batteryVoltageV = params.supplyVoltageV;

        MazeMap::EncoderObs encoderObservation{};
        encoderObservation.totalLeftCounts = 0;
        encoderObservation.totalRightCounts = 0;
        encoderObservation.omegaLeftRadps = 0.0f;
        encoderObservation.omegaRightRadps = 0.0f;

        MazeMap::ImuAccelObs accelObservation{};
        accelObservation.valid = true;
        accelObservation.accelBodyXMps2 = 0.0f;
        accelObservation.accelBodyYMps2 = 0.0f;

        for (uint32_t index = 0U; index < kOpenFloorUkfBenchmarkWarmupIterations; ++index)
        {
            if (!ExecuteOpenFloorStationaryMeasurementCycle(
                    ukf,
                    kOpenFloorUkfBenchmarkDtSeconds,
                    control,
                    encoderObservation,
                    accelObservation,
                    kOpenFloorUkfBenchmarkStationaryGyroRawRadps))
            {
                std::cerr << "Open-floor UKF warmup failed at iteration " << index << "\n";
                return 1;
            }
            if ((index % (kOpenFloorUkfBenchmarkWarmupIterations / 100)) == 0)
            {
                std::cout << index << "\n";
            }
        }

        ResetOpenFloorBenchmarkUkf(ukf);
        const auto start = std::chrono::steady_clock::now();
        for (uint32_t index = 0U; index < iterations; ++index)
        {
            if (!ExecuteOpenFloorStationaryMeasurementCycle(
                    ukf,
                    kOpenFloorUkfBenchmarkDtSeconds,
                    control,
                    encoderObservation,
                    accelObservation,
                    kOpenFloorUkfBenchmarkStationaryGyroRawRadps))
            {
                std::cerr << "Open-floor UKF benchmark failed at iteration " << index << "\n";
                return 1;
            }
        }
        const auto end = std::chrono::steady_clock::now();

        const std::chrono::duration<double, std::milli> elapsedMs = end - start;
        const double microsecondsPerIteration =
            (iterations > 0U) ?
            ((elapsedMs.count() * 1000.0) / static_cast<double>(iterations)) :
            0.0;
        const MazeMap::VehicleState::StateVector& state = ukf.ukf().state();
        const MazeMap::VehicleState::StateMatrix covariance = ukf.ukf().covariance();

        std::cout << std::fixed << std::setprecision(6);
        std::cout << "Open-floor UKF stationary benchmark\n";
        std::cout << "  measurement_set: predict + encoder_pair + yaw_rate + planar_accel\n";
        std::cout << "  iterations: " << iterations << "\n";
        std::cout << "  dt_seconds: " << kOpenFloorUkfBenchmarkDtSeconds << "\n";
        std::cout << "  fan_duty: " << control.fanDutyCycle << "\n";
        std::cout << "  raw_gyro_radps: " << kOpenFloorUkfBenchmarkStationaryGyroRawRadps << "\n";
        std::cout << "  elapsed_ms: " << elapsedMs.count() << "\n";
        std::cout << "  us_per_iteration: " << microsecondsPerIteration << "\n";
        std::cout << "  final_state:"
            << " px=" << state(MazeMap::VehicleState::kPx)
            << " py=" << state(MazeMap::VehicleState::kPy)
            << " psi=" << state(MazeMap::VehicleState::kPsi)
            << " u=" << state(MazeMap::VehicleState::kU)
            << " v=" << state(MazeMap::VehicleState::kV)
            << " r=" << state(MazeMap::VehicleState::kR)
            << " omega_l=" << state(MazeMap::VehicleState::kOmegaL)
            << " omega_r=" << state(MazeMap::VehicleState::kOmegaR)
            << " bgz=" << state(MazeMap::VehicleState::kBgz)
            << "\n";
        std::cout << "  covariance_diag:"
            << " px=" << covariance(MazeMap::VehicleState::kPx, MazeMap::VehicleState::kPx)
            << " py=" << covariance(MazeMap::VehicleState::kPy, MazeMap::VehicleState::kPy)
            << " psi=" << covariance(MazeMap::VehicleState::kPsi, MazeMap::VehicleState::kPsi)
            << " u=" << covariance(MazeMap::VehicleState::kU, MazeMap::VehicleState::kU)
            << " v=" << covariance(MazeMap::VehicleState::kV, MazeMap::VehicleState::kV)
            << " r=" << covariance(MazeMap::VehicleState::kR, MazeMap::VehicleState::kR)
            << " omega_l=" << covariance(MazeMap::VehicleState::kOmegaL, MazeMap::VehicleState::kOmegaL)
            << " omega_r=" << covariance(MazeMap::VehicleState::kOmegaR, MazeMap::VehicleState::kOmegaR)
            << " bgz=" << covariance(MazeMap::VehicleState::kBgz, MazeMap::VehicleState::kBgz)
            << "\n";
        return 0;
    }
}

void MovePath(const MazeMap::HalfStepPath<PATH_SIZE*2>& path)
{
    int i = 0;
    if (path[0] == simVehicle.GetPosition())
    {
        ++i;
    }
    if (path.GetSize() == i+1)
    {
        //simVehicle.MoveTo(path[0]);
        return;
    }

    MazeMap::Direction d = path[i].DirectionTo(path[i+1]);

    MazeMap::MazeLocation current = path[0];

    int stepsToTake = 1;
    for (uint8_t i = 1; i < path.GetSize(); i++)
    {
        /*while ((i< (path.GetSize() - 1)) && (d == path[i].DirectionTo(path[i + 1])))
        {
            ++stepsToTake;
            ++i;
        }*/
        /*{
            simVehicle.Move(d, stepsToTake);
            d = path[i].DirectionTo(path[i + 1]);
            stepsToTake = 1;
        }*/
        MazeMap::MazeLocation next = path[i];

        simVehicle.MoveTo(next);

    }
        //Sleep(1000);
}


std::string ToString(MazeMap::ManeuverCode code)
{
    switch (code)
    {
    case MazeMap::MC_NONE: return std::string("MC_NONE");
    case MazeMap::S1: return std::string("S01");
    case MazeMap::S2: return std::string("S02");
    case MazeMap::S3: return std::string("S03");
    case MazeMap::S4: return std::string("S04");
    case MazeMap::S5: return std::string("S05");
    case MazeMap::S6: return std::string("S06");
    case MazeMap::S7: return std::string("S07");
    case MazeMap::S8: return std::string("S08");
    case MazeMap::S9: return std::string("S09");
    case MazeMap::S10: return std::string("S10");
    case MazeMap::S11: return std::string("S11");
    case MazeMap::S12: return std::string("S12");
    case MazeMap::S13: return std::string("S13");
    case MazeMap::S14: return std::string("S14");
    case MazeMap::S15: return std::string("S15");
    case MazeMap::S16: return std::string("S16");
    case MazeMap::S17: return std::string("S17");
    case MazeMap::S18: return std::string("S18");
    case MazeMap::S19: return std::string("S19");
    case MazeMap::S20: return std::string("S20");
    case MazeMap::S21: return std::string("S21");
    case MazeMap::S22: return std::string("S22");
    case MazeMap::S23: return std::string("S23");
    case MazeMap::S24: return std::string("S24");
    case MazeMap::S25: return std::string("S25");
    case MazeMap::S26: return std::string("S26");
    case MazeMap::S27: return std::string("S27");
    case MazeMap::S28: return std::string("S28");
    case MazeMap::S29: return std::string("S29");
    case MazeMap::S30: return std::string("S30");
    case MazeMap::S31: return std::string("S31");

    case MazeMap::IP45: return std::string("IP45");
    case MazeMap::IP90: return std::string("IP90");
    case MazeMap::IP135: return std::string("IP135");
    case MazeMap::IP180: return std::string("IP180");
    case MazeMap::S45SS: return std::string("S45SS");
    case MazeMap::S45SD: return std::string("S45SD");
    case MazeMap::S45LS: return std::string("S45LS");
    case MazeMap::S45LD: return std::string("S45LD");
    case MazeMap::S90SS: return std::string("S90SS");
    case MazeMap::S90LS: return std::string("S90LS");
    case MazeMap::S90SD: return std::string("S90SD");
    case MazeMap::S90LD: return std::string("S90LD");
    case MazeMap::S135SS: return std::string("S135SS");
    case MazeMap::S135LS: return std::string("S135LS");
    case MazeMap::S135SD: return std::string("S135SD");
    case MazeMap::S135LD: return std::string("S135LD");
    case MazeMap::S180SS: return std::string("S180SS");
    case MazeMap::S180LS: return std::string("S180LS");
    case MazeMap::S90ELD: return std::string("S90ELD");
    case MazeMap::S180ELS: return std::string("S180ELS");

    case MazeMap::IP45_M:return std::string("IP45_M");
    case MazeMap::IP90_M:return std::string("IP90_M");
    case MazeMap::IP135_M:return std::string("IP135_M");
    case MazeMap::IP180_M:return std::string("IP180_M");
    case MazeMap::S45SD_M:return std::string("S45SD_M");
    case MazeMap::S45SS_M:return std::string("S45SS_M");
    case MazeMap::S45LS_M:return std::string("S45LS_M");
    case MazeMap::S45LD_M:return std::string("S45LD_M");
    case MazeMap::S90SS_M:return std::string("S90SS_M");
    case MazeMap::S90SD_M:return std::string("S90SD_M");
    case MazeMap::S90LS_M:return std::string("S90LS_M");
    case MazeMap::S90LD_M:return std::string("S90LD_M");
    case MazeMap::S135SS_M:return std::string("S135SS_M");
    case MazeMap::S135SD_M:return std::string("S135SD_M");
    case MazeMap::S135LS_M:return std::string("S135LS_M");
    case MazeMap::S135LD_M:return std::string("S135LD_M");
    case MazeMap::S180SS_M:return std::string("S180SS_M");
    case MazeMap::S180LS_M:return std::string("S180LS_M");
    case MazeMap::S90ELD_M:return std::string("S90ELD_M");
    case MazeMap::S180ELS_M:return std::string("S180ELS_M");
    default:
        return std::string("");
    }
}
void MovePath(const MazeMap::ManeuverPath& path)
{
    MazeMap::ManeuverSet& ms = MazeMap::ManeuverSet::GetSet();

    for (int i = 0; i < path.GetSize(); ++i)
    {
        if (path[i] == MazeMap::MC_NONE)
        {
            break;
        }
        if (path[i] < MazeMap::S31)
        {
            simVehicle.Move(MazeMap::RelativeDirection::Forward, path[i]);
        }
        else
        {
            const MazeMap::Maneuver& man = ms[path[i]];

            for (uint8_t j = 0; j < man.GetStepCount(); j++)
            {
                MazeMap::RelativeDirectionalDistance rdd = man.GetStep(j);
                if (path[i] & MazeMap::MIRRORED_MANEUVER_FLAG)
                {
                    rdd = MazeMap::RelativeDirectionalDistance(-rdd.GetDirection(), rdd.GetDistance());
                }
                simVehicle.Move(rdd.GetDirection(), rdd.GetDistance());
            }
        }
    }
}

void ReadWalls()
{
    MazeMap::Cell& c = maze[simVehicle.GetPosition().GetFirstConnectedCell()];


    for (MazeMap::Direction d = MazeMap::Direction::Up; d <= MazeMap::Direction::Right; d = static_cast<MazeMap::Direction>(d<<1))
    {
        if (c.GetWall(d) == MazeMap::WallState::Unknown)
        {
            bool b = simVehicle.ReadWall(d - simVehicle.GetOrientation());
            if (b)
            {
                maze.SetWall(c, d, MazeMap::WallState::Wall);
            }
            else
            {
                maze.SetWall(c, d, MazeMap::WallState::NoWall);
            }
        }
    }
}

void PostAdvancedWeights()
{

    for (uint8_t i = 0; i < 16; i++)
    {
        for (uint8_t j = 0; j < 16; j++)
        {
            MazeMap::CellCoordinates coords(i, j);
            MazeMap::MazeLocation loc = MazeMap::MazeLocation::CellCenter(coords);
            MazeMap::Direction dMin = MazeMap::Up;
            float minCost = INFINITY;
            for (uint8_t k = 0; k < 8; k++)
            {
                MazeMap::Direction d = MazeMap::OrdinalDirections[k];
                MazeMap::DirectionalLocation dirLoc(loc, d);
                if (superAdvancedPathFinder.GetCost(dirLoc) < minCost)
                {
                    minCost = superAdvancedPathFinder.GetCost(dirLoc);
                    dMin = d;
                }
            }
            MazeMap::DirectionalLocation samplePoint(loc, dMin);
            std::stringstream ss = std::stringstream();
            //ss << ToString(superAdvancedPathFinder.GetCode(samplePoint));
            ss << (int)(1000 * superAdvancedPathFinder.GetCost(samplePoint));
            API::setText(i, j, ss.str());
        }
    }
}

void profile();
void runMMSSim();
int main(int argc, char* argv[])
{
    RunOpenFloorUkfStationaryBenchmark(10000000);
    return 0;
    if (argc > 1 && IsOpenFloorUkfBenchmarkArg(argv[1]))
    {
        return RunOpenFloorUkfStationaryBenchmark(
            (argc > 2) ? ParseBenchmarkIterations(argv[2]) : kDefaultOpenFloorUkfBenchmarkIterations);
    }

    runMMSSim();
    return 0;
}
void profile()
{
    MazeMap::Mazes::SetupMazes();
    MazeMap::Maze& apec = MazeMap::Mazes::GetMazeAPEC2016();
    for (uint8_t i = 0; i < 16; i++)
    {
        for (uint8_t j = 0; j < 16; j++)
        {
            for (MazeMap::Direction d = MazeMap::Up; d <= MazeMap::Right; d=d<<1)
            {
                maze(i, j).SetWall(d, apec(i, j).GetWall(d));
            }
        }
    }

    MazeMap::CellCoordinates start(0, 0);
    MazeMap::Direction startDirection = MazeMap::Up;

    MazeMap::ManeuverPath mp = MazeMap::ManeuverPath();

    for (size_t i = 0; i < PROCESSING_CYCLES; i++)
    {
        superAdvancedPathFinder.ManeuverPathToGoal(start, startDirection, mp);
    }

}

void runMMSSim()
{
    MazeMap::FloodFillPathFinder pathfinder = MazeMap::FloodFillPathFinder(maze, vehicle);
    MazeMap::HalfStepPath<PATH_SIZE*2> path = MazeMap::HalfStepPath<PATH_SIZE * 2>();
    MazeMap::Direction d = MazeMap::Direction::Up;

    Sleep(1000);
    for (int i = 0; i < 16; ++i)
    {
        API::setWall(0, i, 'w');
        API::setWall(i, 15, 'n');
        API::setWall(15, 15-i, 'e');
        API::setWall(15-i, 0, 's');
    }
    int n = 0;
    try
    {
        bool goalfound = false;
        while (!maze.IsComplete())
        {
            pathfinder.HalfStepPathToNearestUnknown(simVehicle.GetPosition().GetFirstConnectedCell(), simVehicle.GetOrientation(), path);
            MazeMap::MazeLocation loc1 = simVehicle.GetPosition();
            MovePath(path);
            MazeMap::MazeLocation loc2 = simVehicle.GetPosition();
            //std::cout << "moved " << (int)simVehicle.GetPosition().GetX() << ", " << (int)simVehicle.GetPosition().GetY() << std::endl;
            ReadWalls();
            if (!goalfound)
            {
                if (maze.HasFoundGoal())
                {
                    goalfound = true;
                    MazeMap::CellCoordinates goal = maze.GetGoalLowerLeft();
                    API::setColor(goal.GetX(),   goal.GetY(), 'b');
                    API::setColor(goal.GetX()+1, goal.GetY(), 'b');
                    API::setColor(goal.GetX(),   goal.GetY()+1, 'b');
                    API::setColor(goal.GetX()+1, goal.GetY()+1, 'b');
                    Sleep(1000);
                }
            }
            ++n;
            //std::cout << n << "\n";

        }
        maze.ExportToFile("MazeText.txt");
        pathfinder.HalfStepPathFromTo(simVehicle.GetPosition().GetFirstConnectedCell(), simVehicle.GetOrientation(), MazeMap::CellCoordinates(0, 0), path);
        MovePath(path);
        simVehicle.Turn(MazeMap::Direction::Up - simVehicle.GetOrientation());

        maze.PreCalculate();



        pathfinder.HalfStepPathToGoal(simVehicle.GetPosition().GetFirstConnectedCell(), simVehicle.GetOrientation(), path);

        path.ConvertToPath(p);

        for (uint16_t i = 0; i < p.GetSize(); i++)
        {
            API::setColor(p[i].GetX(), p[i].GetY(), 'y');
        }
        MazeMap::ManeuverPath mpath = MazeMap::ManeuverPath();
        //for (size_t i = 0; i < PROCESSING_CYCLES; i++)
        {
            superAdvancedPathFinder.ManeuverPathToGoal(simVehicle.GetPosition().GetFirstConnectedCell(), simVehicle.GetOrientation(), mpath);

        }

        PostAdvancedWeights();
        std::cout << superAdvancedPathFinder.GetLastEstimatedTime()<<"\n";
        path.clear();
        mpath.ToHalfStepPath(MazeMap::DirectionalLocation(1, 1, MazeMap::Up), path);
        path.ConvertToPath(p);

        for (uint16_t i = 0; i < p.GetSize(); i++)
        {
            API::setColor(p[i].GetX(), p[i].GetY(), 'g');
        }

        MovePath(mpath);
        simVehicle.Turn(MazeMap::Reverse);
        Sleep(1000);
        mpath.clear();
        superAdvancedPathFinder.ManeuverPathFromTo(simVehicle.GetPosition().GetFirstConnectedCell(), simVehicle.GetOrientation(), MazeMap::CellCoordinates(0, 0), mpath);

        PostAdvancedWeights();

        /*path.clear();
        mpath.ToHalfStepPath(MazeMap::DirectionalLocation(simVehicle.GetPosition(), simVehicle.GetOrientation()), path);
        path.ConvertToPath(p);

        for (size_t i = 0; i < p.GetSize(); i++)
        {
            API::setColor(p[i].GetX(), p[i].GetY(), 'o');
        }*/
        MovePath(mpath);
        //pathfinder.HalfStepPathFromTo(simVehicle.GetPosition().GetFirstConnectedCell(), simVehicle.GetOrientation(), MazeMap::CellCoordinates(0, 0), path);

        //MovePath(path);
    }
    catch (int errorCode)
    {
        std::cout << errorCode << "\n";
    }
    catch (const std::exception& e)
    {
        std::cout << e.what() << "\n";
    }
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file




