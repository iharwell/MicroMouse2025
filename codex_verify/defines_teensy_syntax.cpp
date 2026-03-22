#define ARDUINO_TEENSY41 1
#include "../MazeMap/MazeMap/Defines.h"
#include "../MazeMap/MazeMap/MotorEncoderDrive.h"

class EXPORT Dummy
{
public:
    uint8_t value = 0;
};

int main()
{
    Dummy dummy;
    MazeMap::MotorEncoderDrive drive;
    drive.getDriveCommand();
    return static_cast<int>(dummy.value);
}
