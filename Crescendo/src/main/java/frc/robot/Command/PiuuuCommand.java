package frc.robot.Command;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.PiuuuSubsystem;
  
public class PiuuuCommand extends Command {
//Nombre de los comandos ara llamar en e container 
  private final PiuuuSubsystem piuuuSubsystem;
  private Supplier<Boolean> Sol1, Sol2, Agr1, Agr2, Disp1, Disp2;

//Declaracion del los comandos en el CommandBase 
  public PiuuuCommand(PiuuuSubsystem piuuuSubsystem, Supplier<Boolean> Sol1, Supplier<Boolean> Sol2, 
  Supplier<Boolean> Agr1, Supplier<Boolean> Agr2, Supplier<Boolean> Disp1, Supplier<Boolean> Disp2){
    this.Agr1 = Agr1;
    this.Agr2 = Agr2;
    this.Sol1 = Sol1;
    this.Sol2 = Sol2;
    this.Disp1 = Disp1;
    this.Disp2 = Disp2;

    this.piuuuSubsystem = piuuuSubsystem;
    addRequirements(piuuuSubsystem);
  }

//Inciliacion de Metodos del Subsistema (init)
  @Override
  public void initialize() {}

//Ejecucion de los metodos del subsistema (periodic)
  @Override
  public void execute() {
    if(Sol1.get()) {
      piuuuSubsystem.MtrItkVel(1.0);

    } else if (Sol2.get()) {
      piuuuSubsystem.MtrItkVel(0.5);

    } else if (Agr1.get()) {
      piuuuSubsystem.MtrItkVel(-1.0);

    } else if (Agr2.get()){
      piuuuSubsystem.MtrItkVel(-0.5);

    } else {
      piuuuSubsystem.MtrItkVel(0);
    }

    if(Disp1.get()){
      piuuuSubsystem.MtrShtVel(1);

    }else if(Disp2.get()){
      piuuuSubsystem.MtrShtVel(-1);

    }else{
      piuuuSubsystem.MtrShtVel(0);
    }
  }

//Finalisacion de los metodos del Subsistema
  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
