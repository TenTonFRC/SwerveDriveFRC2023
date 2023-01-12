#pragma once

#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <wpi/numbers>

/**
 * This header contains hold robot-wide numerical or boolean constants ONLY.
 * 
 * Place constants into subsystem/command -specific NAMESPACES within this
 * header, which can then be included (where they are needed).
 */

namespace controllerConstants {
    //USB port addresses on drivestation PC.
    constexpr int kControllerMainID = 0;
    constexpr int kControllerAuxID = 1;
}

namespace drivetrainConstants {
    //CAN IDs
    constexpr int kMotorDriveFrontRightID = 1;
    constexpr int kMotorDriveRearRightID = 4;
    constexpr int kMotorDriveFrontLeftID = 3;
    constexpr int kMotorDriveRearLeftID = 2;

    constexpr int kMotorTurnFrontRightID = 5;
    constexpr int kMotorTurnRearRightID = 8;
    constexpr int kMotorTurnFrontLeftID = 7;
    constexpr int kMotorTurnRearLeftID = 6;

    constexpr int kEncoderTurnFrontRightID = 9;
    constexpr int kEncoderTurnRearRightID = 12;
    constexpr int kEncoderTurnFrontLeftID = 11;
    constexpr int kEncoderTurnRearLeftID = 10;

    constexpr double kFrontRight{0};
    constexpr double kRearRight{0};
    constexpr double kFrontLeft{0};
    constexpr double kRearLeft{0};


    namespace swerveModules {
        constexpr double kModuleFrontRight[4]{kMotorDriveFrontRightID,
                                                   kMotorTurnFrontRightID,
                                                   kEncoderTurnFrontRightID,
                                                   kFrontRight};
        constexpr double kModuleRearRight[4]{kMotorDriveRearRightID,
                                                  kMotorTurnRearRightID,
                                                  kEncoderTurnRearRightID,
                                                  kRearRight};
        constexpr double kModuleFrontLeft[4]{kMotorDriveFrontLeftID,
                                                  kMotorTurnFrontLeftID,
                                                  kEncoderTurnFrontLeftID,
                                                  kFrontLeft};
        constexpr double kModuleRearLeft[4]{kMotorDriveRearLeftID,
                                                 kMotorTurnRearLeftID,
                                                 kEncoderTurnRearLeftID,
                                                 kRearLeft};
    }

    namespace calculations {
        constexpr auto kFinalDriveRatio{6.75 * 360_deg};
        constexpr units::length::inch_t kWheelCircumference = {2 * wpi::numbers::pi * 3.8_in / 2};

        constexpr auto kModuleMaxSpeed{16.3_fps};
        constexpr auto kChassisMaxSpeed{16.3_fps};

        constexpr auto kModuleMaxAngularVelocity{wpi::numbers::pi * 1_rad_per_s};  // radians per second
        constexpr auto kModuleMaxAngularAcceleration{wpi::numbers::pi * 2_rad_per_s / 1_s};  // radians per second^2

        constexpr double kMotorMaxOutput = 0.5;
        constexpr double kMotorDeadband = 0.1;
    }
}
