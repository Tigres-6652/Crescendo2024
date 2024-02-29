package frc.robot.Command;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ArmSubsystem;

public class ArmCommand extends Command {

  private final ArmSubsystem armSubsystem;
  private Supplier<Boolean> A_de_Arriba, A_de_Abajo, Desbloqueo;
  private Supplier<Double> GodMovent;
  
  public ArmCommand (ArmSubsystem armSubsystem, Supplier<Boolean> A_de_Arriba, Supplier<Boolean> A_de_Abajo, Supplier <Double> GodMovent, Supplier <Boolean> Desbloqueo){
  this.A_de_Arriba = A_de_Arriba;
  this.A_de_Abajo = A_de_Abajo;
  this.Desbloqueo = Desbloqueo;
  this.GodMovent = GodMovent;

  this.armSubsystem = armSubsystem;
  addRequirements(armSubsystem);

  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
      armSubsystem.manejolibre(GodMovent.get());

  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
