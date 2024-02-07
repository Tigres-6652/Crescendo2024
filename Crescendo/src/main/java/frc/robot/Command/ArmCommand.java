package frc.robot.Command;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ArmSubsystem;

public class ArmCommand extends Command {

  private final ArmSubsystem armSubsystem;
  private Supplier<Boolean> FunArm, Boton_0, Boton_90;

public ArmCommand (ArmSubsystem armSubsystem, Supplier<Boolean> FunArm, Supplier<Boolean> Boton_0, Supplier<Boolean> Boton_90){
  this.Boton_0 = Boton_0;
  this.Boton_90 = Boton_90;
  this.FunArm = FunArm;
  this.armSubsystem = armSubsystem;
  addRequirements(armSubsystem);

}

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    armSubsystem.Botones(Boton_0.get(), Boton_90.get());

  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
