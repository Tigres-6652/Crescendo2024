package frc.robot.Command.AutoCmd;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Command.ActnCmd.Acomodo_Y_Disparo;
import frc.robot.Command.ActnCmd.Acomodo_Y_Recarga;
public class Autonomo1 extends SequentialCommandGroup {

  public Autonomo1() {

    addCommands(
      new ParallelDeadlineGroup(new WaitCommand(2.0) , new SequentialCommandGroup(new Acomodo_Y_Recarga())),
      new ParallelDeadlineGroup(new WaitCommand(2.0) , new SequentialCommandGroup(new Acomodo_Y_Disparo()))
    );
  }
}
