// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Command;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ArmSubsystem;

public class ArmCommand extends Command {

  private final ArmSubsystem armSubsystem;
  private Supplier<Boolean> FunArm;

public ArmCommand (ArmSubsystem armSubsystem, Supplier<Boolean> FunArm){
  this.FunArm = FunArm;
  this.armSubsystem = armSubsystem;
  addRequirements(armSubsystem);

}

  @Override
  public void initialize() {
    armSubsystem.TalonFXConfigs();
  }

  @Override
  public void execute() {

    armSubsystem.control();

    
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
