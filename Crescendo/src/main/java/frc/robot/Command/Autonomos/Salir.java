package frc.robot.Command.Autonomos;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Command.DriveCommand;

public class Salir extends SequentialCommandGroup {

  public Salir() {

    addCommands(new ParallelDeadlineGroup(new WaitCommand(3), new SequentialCommandGroup(new DriveCommand(RobotContainer.driveSubsystem, ()-> 0.5, ()-> 0.0))));
  }
}
