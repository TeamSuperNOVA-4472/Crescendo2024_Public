// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.SimMechanismCommand;
import frc.robot.subsystems.SimMechanism;

public class RobotContainer {

  private final XboxController mMechController = new XboxController(0);
  private final SimMechanism mSimMechanism = new SimMechanism();
  private final SimMechanismCommand mSimMechanismCommand = new SimMechanismCommand(
    () -> -mMechController.getLeftY(),
    () -> -mMechController.getRawAxis(3),
    mSimMechanism);

  public RobotContainer() {
    mSimMechanism.setDefaultCommand(mSimMechanismCommand);
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
