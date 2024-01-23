// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Command.ArmCommand;
import frc.robot.Subsystems.ArmSubsystem;

public class RobotContainer {

  private final ArmSubsystem armSubsystem = new ArmSubsystem();

  private final Joystick Ctrl = new Joystick(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {

  armSubsystem.setDefaultCommand(new ArmCommand(armSubsystem, () -> Ctrl.getRawButton(1)));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
