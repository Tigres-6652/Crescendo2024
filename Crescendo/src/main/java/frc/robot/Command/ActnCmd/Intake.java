package frc.robot.Command.ActnCmd;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class Intake extends Command {

  public Intake() {}

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    RobotContainer.intakeSubsystem.MtrItkVel(0.6);
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.intakeSubsystem.MtrItkVel(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
