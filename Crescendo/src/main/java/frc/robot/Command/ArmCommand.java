package frc.robot.Command;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ArmSubsystem;

public class ArmCommand extends Command {

  private final ArmSubsystem armSubsystem;
  private Supplier<Boolean> Boton_0, Boton_90;
  private Supplier<Boolean> PaAbajo, PaEnMedio, RtrnHm;

public ArmCommand (ArmSubsystem armSubsystem, Supplier<Boolean> Boton_0, Supplier<Boolean> Boton_90, 
Supplier<Boolean> PaAbajo, Supplier<Boolean> PaEnMedio, Supplier<Boolean> RtrnHm){
  this.PaAbajo = PaAbajo;
  this.PaEnMedio = PaEnMedio;
  this.RtrnHm = RtrnHm;
  
  this.Boton_0 = Boton_0;
  this.Boton_90 = Boton_90;
  this.armSubsystem = armSubsystem;
  addRequirements(armSubsystem);

}

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    armSubsystem.Botones(Boton_0.get(), Boton_90.get());

    if (PaAbajo.get()) {
      armSubsystem.ArmAxisPulses(PaAbajo.MtrMotionAndLimit);

    }else if (PaEnMedio.get()){
      armSubsystem.ArmAxisPulses(PaEnMedio.MtrMotionAndLimit);

    }else if (RtrnHm.get()){
      armSubsystem.RtrnHm(RtrnHm.get());

    }else{
      armSubsystem.MtrMotionAndLimit(0);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
