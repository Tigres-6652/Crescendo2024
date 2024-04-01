package frc.robot.Command.ActnCmd;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class Acomodo_Y_Disparo extends Command {

  public Acomodo_Y_Disparo() {}

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    RobotContainer.piuuuSubsystem.ShootRPM(50);
    RobotContainer.driveSubsystem.AimAndDist(1);
    RobotContainer.intakeSubsystem.MtrItkVel(-0.6);
    RobotContainer.armSubsystem.anguloVariable();
  }
  
  @Override
  public void end(boolean interrupted) {
    RobotContainer.piuuuSubsystem.ShootRPM(50);
    RobotContainer.driveSubsystem.AimAndDist(1);
    RobotContainer.intakeSubsystem.MtrItkVel(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}