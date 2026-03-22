#include "../MazeMap/MazeMap/Defines.h"
#include "../MazeMap/MazeMap/MotorEncoderDrive.h"
#include "../MazeMap/MazeMap/WheelMotor.h"
#include "../MazeMap/MazeMap/WallSensor.h"

int main()
{
    MazeMap::MotorEncoderDrive drive;
    return drive.getEncoderCount();
}
