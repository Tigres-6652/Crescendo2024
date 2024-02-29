package frc.robot.Command;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DriveSubsystem;

public class DriveCommand extends Command {

  private final DriveSubsystem driveSubsystem;
  private Supplier<Double> FunSpd, FunGrr;

  public DriveCommand(DriveSubsystem driveSubsystem, Supplier<Double> FunSpd, Supplier<Double> FunSpn) {
    this.FunGrr = FunSpn;
    this.FunSpd = FunSpd;
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    driveSubsystem.Inverte();
    driveSubsystem.FollowMotor();
  }

  @Override
  public void execute() {
    driveSubsystem.Arcade_Drive(FunSpd.get(), FunGrr.get());
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.Arcade_Drive(0, 0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
