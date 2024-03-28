// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Command;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
  /** Creates a new IntakeCommand. */


private Supplier<Boolean> Sol1, Sol2, Agr1, Agr2;

IntakeSubsystem intakeSubsystem;
  public IntakeCommand(IntakeSubsystem intakeSubsystem,Supplier<Boolean> Sol1, Supplier<Boolean> Sol2, 
  Supplier<Boolean> Agr1, Supplier<Boolean> Agr2) {

    this.Agr1 = Agr1;
    this.Agr2 = Agr2;
    this.Sol1 = Sol1;
    this.Sol2 = Sol2;
    this.intakeSubsystem=intakeSubsystem;
    addRequirements(intakeSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(Sol1.get()){

            intakeSubsystem.MtrItkVel(-0.6);


    }else if(Sol2.get()){

      intakeSubsystem.MtrItkVel(-0.4);


    }else if(Agr1.get()){

intakeSubsystem.MtrItkVel(0.6);

    }else if(Agr2.get()){

      intakeSubsystem.MtrItkVel(0.4);


    }else{

      intakeSubsystem.MtrItkVel(0);


    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    intakeSubsystem.MtrItkVel(0);


  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
