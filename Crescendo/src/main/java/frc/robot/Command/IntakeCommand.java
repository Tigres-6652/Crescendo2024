package frc.robot.Command;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.IntakeSubsystem;
  
public class IntakeCommand extends Command {
//Nombre de los comandos ara llamar en e container 
  private final IntakeSubsystem intakeSubsystem;
  private Supplier<Boolean> Sol1, Sol2, Agr1, Agr2;

//Declaracion del los comandos en el CommandBase 
  public IntakeCommand(IntakeSubsystem intakeSubsystem, Supplier<Boolean> Sol1, Supplier<Boolean> Sol2, Supplier<Boolean> Agr1, Supplier<Boolean> Agr2){
    this.Agr1 = Agr1;
    this.Agr2 = Agr2;
    this.Sol1 = Sol1;
    this.Sol2 = Sol2;

    this.intakeSubsystem = intakeSubsystem;
    addRequirements(intakeSubsystem);
  }

//Inciliacion de Metodos del Subsistema (init)
  @Override
  public void initialize() {}

//Ejecucion de los metodos del subsistema (periodic)
  @Override
  public void execute() {
    if(Sol1.get()) {
      intakeSubsystem.MtrItkVel(1.0);

    } else if (Sol2.get()) {
      intakeSubsystem.MtrItkVel(0.5);

    } else if (Agr1.get()) {
      intakeSubsystem.MtrItkVel(-1.0);

    } else if (Agr2.get()){
      intakeSubsystem.MtrItkVel(-0.5);

    } else {
      intakeSubsystem.MtrItkVel(0);

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
