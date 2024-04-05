package frc.robot.Command;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.PiuuuSubsystem;
  
public class PiuuuCommand extends Command {
//Nombre de los comandos ara llamar en e container 
  private final PiuuuSubsystem piuuuSubsystem;
  private Supplier<Boolean>  Disp5, Disp50, Disp25;

//Declaracion del los comandos en el CommandBase 
  public PiuuuCommand(PiuuuSubsystem piuuuSubsystem, Supplier<Boolean> Disp5, Supplier<Boolean> Disp50, Supplier<Boolean> Disp25){
    this.Disp5 = Disp5;
    this.Disp50 = Disp50;
    this.Disp25 = Disp25;
    this.piuuuSubsystem = piuuuSubsystem;
    addRequirements(piuuuSubsystem);
  }

//Inciliacion de Metodos del Subsistema (init)
  @Override
  public void initialize() {
    piuuuSubsystem.configPID();

  }

//Ejecucion de los metodos del subsistema (periodic)
  @Override
  public void execute() {
    

    if(Disp5.get()){
      piuuuSubsystem.ShootRPM(5);

    }else if(Disp50.get()){
      piuuuSubsystem.ShootRPM(40);

    }else if(Disp25.get()){
      piuuuSubsystem.ShootRPM(25);

    }else{
      piuuuSubsystem.MtrShtVel(0);
    }
  }

//Finalisacion de los metodos del Subsistema
  @Override
  public void end(boolean interrupted) {
      piuuuSubsystem.MtrShtVel(0);

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}