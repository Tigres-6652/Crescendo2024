package frc.robot.Command;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DriveSubsystem;

public class DriveCommand extends Command {
//Nombre de los comandos ara llamar en e container 
  private final DriveSubsystem driveSubsystem;
  private Supplier<Double> FunSpd, FunGrr;
  private Supplier<Boolean> autoapuntado;

//Declaracion del los comandos en el CommandBase 
  public DriveCommand(DriveSubsystem driveSubsystem, Supplier<Double> FunSpd, Supplier<Double> FunGrr, Supplier<Boolean> autoapuntado) {
    this.FunGrr = FunGrr;
    this.FunSpd = FunSpd;
    this.autoapuntado=autoapuntado;
    this.driveSubsystem = driveSubsystem;
    
    addRequirements(driveSubsystem);
  }

//Inciliacion de Metodos del Subsistema (init)
  @Override
  public void initialize() {
    driveSubsystem.configtalon();    
  //  driveSubsystem.Lime_Light();
    driveSubsystem.configprioritylime();
  }

//Ejecucion de los metodos del subsistema (periodic)
  @Override
  public void execute() {
    if(autoapuntado.get()){
      driveSubsystem.AimAndDist(FunSpd.get());

    }else{
    driveSubsystem.Arcade_Drive(FunSpd.get(), FunGrr.get());
    }
    driveSubsystem.Smartdashboard();
  }

//Finalisacion de los metodos del Subsistema
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.Arcade_Drive(0, 0);
    driveSubsystem.Reset();
    //driveSubsystem.resetCordenadas();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
