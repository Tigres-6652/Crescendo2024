package frc.robot.Command;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ArmSubsystem;

public class ArmCommand extends Command {
//Nombre de los comandos ara llamar en e container 
  private final ArmSubsystem armSubsystem;
  private Supplier<Double> GoodMovend;
  private Supplier<Boolean> MovSpeak, MovAmp, MovGrd;

//Declaracion del los comandos en el CommandBase 
  public ArmCommand (ArmSubsystem armSubsystem, Supplier<Double> GoodMovend, Supplier<Boolean> MovSpeak, Supplier<Boolean> MovAmp, Supplier<Boolean> MovGrd){
    this.GoodMovend = GoodMovend;
    this.MovSpeak = MovSpeak;
    this.MovAmp = MovAmp;
    this.MovGrd = MovGrd;
    this.armSubsystem = armSubsystem;
    addRequirements(armSubsystem);
    
  } 

//Inciliacion de Metodos del Subsistema (init)
  @Override
  public void initialize() {}

//Ejecucion de los metodos del subsistema (periodic)
  @Override
  public void execute() {

    if(MovAmp.get()){
      armSubsystem.anguloVariable();

    }else if(MovSpeak.get()){
      armSubsystem.RgtPstnVrbl(14);

    }else if(MovGrd.get()){
      armSubsystem.RgtPstnVrbl(32);

    }else{    
      armSubsystem.MtrInvNFllw(GoodMovend.get());
      
    }
  }

//Finalisacion de los metodos del Subsistema
  @Override
  public void end(boolean interrupted) {
    armSubsystem.MtrInvNFllw(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
