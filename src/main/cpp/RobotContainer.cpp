// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//
#include "RobotContainer.h"
#include <frc/XboxController.h>



RobotContainer::RobotContainer() : m_Auto(&m_drivetrain) {
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureButtonBindings();

 

  m_drivetrain.SetDefaultCommand(Drive(
    &m_drivetrain,
    [this] { return m_controllerMain.GetLeftX(); },
    [this] { return m_controllerMain.GetLeftY(); },
    [this] { return m_controllerMain.GetRightX(); })); 
}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return &m_Auto;
}
//