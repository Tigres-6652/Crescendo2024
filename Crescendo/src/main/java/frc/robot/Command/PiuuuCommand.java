package frc.robot.Command;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.PiuuuSubsystem;
  
public class PiuuuCommand extends Command {
//Nombre de los comandos ara llamar en e container 
  private final PiuuuSubsystem piuuuSubsystem;
  private Supplier<Boolean>  Disp1, Disp2, Disp3;

//Declaracion del los comandos en el CommandBase 
  public PiuuuCommand(PiuuuSubsystem piuuuSubsystem, Supplier<Boolean> Disp1, Supplier<Boolean> Disp2, Supplier<Boolean> Disp3){
    this.Disp1 = Disp1;
    this.Disp2 = Disp2;
    this.Disp3 = Disp3;
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
    

    if(Disp1.get()){
      piuuuSubsystem.ShootRPM(5);

    }else if(Disp2.get()){
      piuuuSubsystem.ShootRPM(40);

    }else if(Disp3.get()){
      piuuuSubsystem.ShootRPM(25);

    }else{
      piuuuSubsystem.MtrShtVel(0);
    }
  }

//Finalisacion de los metodos del Subsistema
  @Override
  public void end(boolean interrupted) {
      piuuuSubsystem.motoroff();

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
