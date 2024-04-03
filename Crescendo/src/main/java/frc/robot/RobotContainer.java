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
import frc.robot.Command.ActnCmd.Acomodo;
import frc.robot.Command.ActnCmd.Intake;
import frc.robot.Command.ActnCmd.Recarga;
import frc.robot.Command.ActnCmd.brazoApuntado;
import frc.robot.Command.ActnCmd.disparar;
import frc.robot.Command.AutoCmd.AimBot;
import frc.robot.Command.AutoCmd.Autonomo1;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.PiuuuSubsystem;

public class RobotContainer {
//llamar los subsistemas 
  public final static DriveSubsystem driveSubsystem = new DriveSubsystem();
  public final static ArmSubsystem armSubsystem = new ArmSubsystem();
  public final static PiuuuSubsystem piuuuSubsystem = new PiuuuSubsystem();
  public final static IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

//declaracion de los controles
  private final Joystick FirstD = new Joystick(0);
  private final Joystick SecondD = new Joystick(1);

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
  configureBindings();

//building the auto chooser for pathplanner
  autoChooser = AutoBuilder.buildAutoChooser();
  SmartDashboard.putData("AutoChooser", autoChooser);
  
  NamedCommands.registerCommand("Acomodo", new InstantCommand);
  NamedCommands.registerCommand("Acomodo", new Acomodo());
  NamedCommands.registerCommand("brazoAuto", new brazoApuntado());
  NamedCommands.registerCommand("disparar", new disparar());
  NamedCommands.registerCommand("intake", new Intake());
  NamedCommands.registerCommand("Recarga", new Recarga());
  NamedCommands.registerCommand("aimbot", new AimBot());
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
    return new Autonomo1();
    //autoChooser.getSelected();
  }
}