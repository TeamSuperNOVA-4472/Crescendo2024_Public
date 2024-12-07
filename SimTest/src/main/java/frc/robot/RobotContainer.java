// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.SimDriveSwerveCommand;
import frc.robot.subsystems.SimDrive;

public class RobotContainer {

  private final XboxController mDriveController = new XboxController(0);
  private final SimDrive mSimDrive = new SimDrive();
  private final SimDriveSwerveCommand mSimDriveCommand = 
    new SimDriveSwerveCommand(
      mSimDrive,
      () -> -mDriveController.getLeftY(),
      () -> -mDriveController.getLeftX(),
      () -> -mDriveController.getRightX());

  public RobotContainer() {
    configureBindings();
    mSimDrive.setDefaultCommand(mSimDriveCommand);
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
