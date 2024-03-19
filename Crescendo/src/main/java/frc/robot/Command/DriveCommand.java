package frc.robot.Command;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DriveSubsystem;

public class DriveCommand extends Command {
//Nombre de los comandos ara llamar en e container 
  private final DriveSubsystem driveSubsystem;
  private Supplier<Double> FunSpd, FunGrr;

//Declaracion del los comandos en el CommandBase 
  public DriveCommand(DriveSubsystem driveSubsystem, Supplier<Double> FunSpd, Supplier<Double> FunGrr) {
    this.FunGrr = FunGrr;
    this.FunSpd = FunSpd;
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
  }

//Inciliacion de Metodos del Subsistema (init)
  @Override
  public void initialize() {
    driveSubsystem.configtalon();
  }

//Ejecucion de los metodos del subsistema (periodic)
  @Override
  public void execute() {
    driveSubsystem.Arcade_Drive(FunSpd.get(), FunGrr.get());
    driveSubsystem.Smartdashboard();

    //driveSubsystem.tanque(FunSpd.get()*12, FunSpd.get()*12);
  }

//Finalisacion de los metodos del Subsistema
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.Arcade_Drive(0, 0);
    driveSubsystem.Reset();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
