package frc.robot.Command.ActnCmd;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class brazoApuntado extends Command {

  public brazoApuntado() {}

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    RobotContainer.armSubsystem.anguloVariable();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
