package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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
    
 NamedCommands.registerCommand("intakeRUN",new InstantCommand( ()-> intakeSubsystem.MtrItkVel(-0.6), driveSubsystem) );
 NamedCommands.registerCommand("intakeSTOP",new InstantCommand( ()-> intakeSubsystem.MtrItkVel(0), driveSubsystem) );


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

//Control del brazo (movimiento libre)
  armSubsystem.setDefaultCommand(new ArmCommand(armSubsystem, 
    ()-> SecondD.getRawAxis(1), ()-> SecondD.getRawButton(6), 
    ()-> SecondD.getRawButton(5), ()-> SecondD.getRawButton(8)));

//Control del Intake (seleccion de velocidades)
  new JoystickButton(SecondD, 1).toggleOnTrue(new IntakeCommand(intakeSubsystem, ()->true, ()->false, ()->false, ()->false));
  new JoystickButton(SecondD, 9).toggleOnTrue(new IntakeCommand(intakeSubsystem, ()->false, ()->false, ()->false, ()->false));
  new JoystickButton(SecondD, 3).toggleOnTrue(new IntakeCommand(intakeSubsystem, ()->false, ()->false, ()->true, ()->false));
  new JoystickButton(SecondD, 10).toggleOnTrue(new IntakeCommand(intakeSubsystem, ()->false, null, ()->false, ()->false));
                                                                                                                                                      //Disp1       Disp2
  new JoystickButton(SecondD, 2).toggleOnTrue(new PiuuuCommand(piuuuSubsystem, () -> true, () -> false));
  new JoystickButton(SecondD, 4).toggleOnTrue(new PiuuuCommand(piuuuSubsystem, () -> false, () -> true));

}

  public Command getAutonomousCommand() {

    return autoChooser.getSelected();
  }
}