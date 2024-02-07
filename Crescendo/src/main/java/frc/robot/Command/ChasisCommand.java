package frc.robot.Command;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ChasisSubsystem;

public class ChasisCommand extends Command {

  private final ChasisSubsystem chasisSubsystem;
  private Supplier<Double> FunSpd, FunGrr;

  public ChasisCommand(ChasisSubsystem chasisSubsystem, Supplier<Double> FunSpd, Supplier<Double> FunGrr) {
    this.FunGrr = FunGrr;
    this.FunSpd = FunSpd;
    this.chasisSubsystem = chasisSubsystem;
    addRequirements(chasisSubsystem);
  }

  @Override
  public void initialize() {
    ChasisSubsystem.Inverte();
    ChasisSubsystem.FollowMotor();
  }

  @Override
  public void execute() {
    chasisSubsystem.Arcade_Drive(FunSpd.get(), FunGrr.get());
  }

  @Override
  public void end(boolean interrupted) {
    chasisSubsystem.Arcade_Drive(0, 0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
