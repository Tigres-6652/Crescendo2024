package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Command.ArmCommand;
import frc.robot.Command.DriveCommand;
import frc.robot.Command.IntakeCommand;
import frc.robot.Command.PiuuuCommand;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.PiuuuSubsystem;

public class RobotContainer {
//llamar los subsistemas 
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final PiuuuSubsystem piuuuSubsystem = new PiuuuSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

//declaracion de los controles
  private final Joystick FirstD = new Joystick(0);
  private final Joystick SecondD = new Joystick(1);

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {

 NamedCommands.registerCommand("intakeSTOP"  ,new InstantCommand(()-> intakeSubsystem.MtrItkVel(0), intakeSubsystem));
 NamedCommands.registerCommand("intakeRUN"   ,new InstantCommand(()-> intakeSubsystem.MtrItkVel(-0.6), intakeSubsystem));
  NamedCommands.registerCommand("intakedesRUN"   ,new InstantCommand(()-> intakeSubsystem.MtrItkVel(0.6), intakeSubsystem));
 NamedCommands.registerCommand( "piuuuRUN"   ,new InstantCommand(()-> piuuuSubsystem.ShootRPM(3), piuuuSubsystem));
 NamedCommands.registerCommand("piuuuSTOP"    ,new InstantCommand(()-> piuuuSubsystem.ShootRPM(0), piuuuSubsystem));
 NamedCommands.registerCommand("ArmRUN"     ,new InstantCommand(()-> armSubsystem.anguloVariable(), armSubsystem));
  NamedCommands.registerCommand("ArmWaitUp"      ,new InstantCommand(()-> armSubsystem.RgtPstnVrbl(16.0), armSubsystem));
 NamedCommands.registerCommand("ArmWait"      ,new InstantCommand(()-> armSubsystem.RgtPstnVrbl(6.0), armSubsystem));
 NamedCommands.registerCommand("AcomodoRUN"  ,new InstantCommand(()-> driveSubsystem.AimAndDist(0), driveSubsystem));
 NamedCommands.registerCommand("aimbot"  ,new InstantCommand(()-> driveSubsystem.AimAndDist(0), driveSubsystem));

  configureBindings();
//building the auto chooser for pathplanner

  autoChooser = AutoBuilder.buildAutoChooser();
  SmartDashboard.putData("AutoChooser", autoChooser);
  }

private void configureBindings() {
//Control del robot
  driveSubsystem.setDefaultCommand(new DriveCommand(driveSubsystem, 
    ()-> (((FirstD.getRawAxis(3)*.40)-(FirstD.getRawAxis(1)*.6))-(FirstD.getRawAxis(2)*.4)),
    ()-> FirstD.getRawAxis(4)*.8, ()-> FirstD.getRawButton(1)));

//Control del Intake (seleccion de velocidades)
  new JoystickButton(FirstD, 3).toggleOnTrue(new IntakeCommand(intakeSubsystem, ()->true, ()->false));
  new JoystickButton(FirstD, 2).toggleOnTrue(new IntakeCommand(intakeSubsystem, ()->false, ()->true));

//Control del brazo (movimiento libre)                                   
  armSubsystem.setDefaultCommand(new ArmCommand(armSubsystem, 
   ()-> SecondD.getRawAxis(1), ()-> SecondD.getRawButton(2), 
   ()-> SecondD.getRawButton(1), ()-> SecondD.getRawButton(3), 
   ()-> SecondD.getRawButton(4)));

//Control de shooter                                                                                                  
  new POVButton(SecondD, 0).toggleOnTrue(new PiuuuCommand(piuuuSubsystem, () -> true, () -> false, () -> false));
  new POVButton(SecondD, 90).toggleOnTrue(new PiuuuCommand(piuuuSubsystem, () -> false, () -> false, () -> true));
  new POVButton(SecondD, 270).toggleOnTrue(new PiuuuCommand(piuuuSubsystem, () -> false, () -> true, () -> false));
}

  public Command getAutonomousCommand() {

    return autoChooser.getSelected();
  }
}