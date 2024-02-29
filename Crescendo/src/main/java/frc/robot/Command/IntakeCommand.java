// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Command;

import java.util.function.Supplier;

import javax.naming.LimitExceededException;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {

  private final IntakeSubsystem intakeSubsystem;
  private Supplier<Double> Intake;
  
  public IntakeCommand(IntakeSubsystem intakeSubsystem, Supplier<Double> Intake){
  this.Intake = Intake;
  this.intakeSubsystem = intakeSubsystem;

  addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    intakeSubsystem.MtrItk(Intake.get());

  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.MtrItk(0);
    
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
