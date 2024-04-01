// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Command.ActnCmd;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class Acomodo extends Command {

  public Acomodo() {}

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    RobotContainer.driveSubsystem.AimAndDist(1);
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.driveSubsystem.AimAndDist(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
