#ifndef MATHSTUFF_H
#define MATHSTUFF_H
#include <cmath>

namespace ControllerIDs {
    //USB port addresses on drivestation PC.
    constexpr int kControllerMainID = 0;
    constexpr int kControllerAuxID = 1;
}

namespace mathConst {
    const double rotationVectorMultiplier = 1.5; // controls how much of the vector addition is dedicated to rotation vs field movement  0 < x < double limit idk
    const double speedLimit = 0.1;             // limit motor speed output   0 < x <= 1
    const double baseWidth = 14.5;             // inches
    const double baseLength = 23.25;
    const double deadband = 0.2;

    // For determining motor positions on an x,y grid
    const double relativeX = baseWidth / 2;
    const double relativeY = baseLength / 2;
}

namespace CanIDs {
    //CAN IDs
    const int kFLDriveMotor = 3;
    const int kFRDriveMotor = 1;
    const int kBLDriveMotor = 2;
    const int kBRDriveMotor = 4;

    // Swerve Motors
    const int kFLSwerveMotor = 7;
    const int kFRSwerveMotor = 5;
    const int kBLSwerveMotor = 6;
    const int kBRSwerveMotor = 8;

    // Encoders
    const int kFLCANCoder = 11;
    const int kFRCANCoder = 9;
    const int kBLCANCoder = 10;
    const int kBRCANCoder = 12;

    namespace offsets {
        constexpr double kFrontRight{-58.05};
        constexpr double kRearRight{58.05};
        constexpr double kFrontLeft{-121.95};
        constexpr double kRearLeft{121.95};
    }
}
#endif