#pragma once

#include <AHRS.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc2/command/SubsystemBase.h>
#include <units/length.h>

#include "swerveModule.h"
#include "Constants.h"

class drivetrain : public frc2::SubsystemBase {
 public:
  drivetrain();

  void SwerveDrive(units::meters_per_second_t xSpeed,
                   units::meters_per_second_t ySpeed,
                   units::radians_per_second_t zRot,
                   bool fieldRelative);

  void UpdateOdometry();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Will be called periodically whenever the CommandScheduler runs during
   * simulation.
   */
  void SimulationPeriodic() override;

 private:

  frc::Translation2d m_locationFrontRight{+7.25_in, -11.625_in};
  frc::Translation2d m_locationRearRight{-7.25_in, -11.625_in};
  frc::Translation2d m_locationFrontLeft{+7.25_in, +11.625_in};
  frc::Translation2d m_locationRearLeft{-7.25_in, +11.625_in};

  swerveModule m_frontRight{drivetrainConstants::swerveModules::kModuleFrontRight};
  swerveModule m_rearRight{drivetrainConstants::swerveModules::kModuleRearRight};
  swerveModule m_frontLeft{drivetrainConstants::swerveModules::kModuleFrontLeft};
  swerveModule m_rearLeft{drivetrainConstants::swerveModules::kModuleRearLeft};

  AHRS m_navX{frc::SPI::kMXP};

  frc::SwerveDriveKinematics<4> m_kinematics{m_locationFrontRight,
                                             m_locationRearRight,
                                             m_locationFrontLeft,
                                             m_locationRearLeft};

  frc::SwerveDriveOdometry<4> m_odometry{m_kinematics, m_navX.GetRotation2d()};
};
