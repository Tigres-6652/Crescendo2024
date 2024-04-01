package frc.robot.Command.AutoCmd;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Command.ArmCommand;
import frc.robot.Command.DriveCommand;
import frc.robot.Command.IntakeCommand;
import frc.robot.Command.PiuuuCommand;

public class AimBot extends SequentialCommandGroup {

  public AimBot() {

    addCommands(  
      new ParallelDeadlineGroup(new WaitCommand(1.5),new SequentialCommandGroup((new DriveCommand(RobotContainer.driveSubsystem, ()-> 0.5, ()-> 0.0, ()-> true)))),
      new ParallelDeadlineGroup(new WaitCommand(2.0),new SequentialCommandGroup((new DriveCommand(RobotContainer.driveSubsystem, ()-> 0.0, ()-> 0.0, ()-> true)))),
      new ParallelDeadlineGroup(new WaitCommand(1.2),new SequentialCommandGroup((new ArmCommand(RobotContainer.armSubsystem, ()-> 0.0, ()-> false, ()-> true, ()-> false, ()-> false)))),
      new ParallelDeadlineGroup(new WaitCommand(1.2),new SequentialCommandGroup((new IntakeCommand(RobotContainer.intakeSubsystem, ()->true, ()->false)))),
      new ParallelDeadlineGroup(new WaitCommand(1.2),new SequentialCommandGroup((new PiuuuCommand(RobotContainer.piuuuSubsystem, () -> false, () -> true, () -> false))))
    );
  }
}
