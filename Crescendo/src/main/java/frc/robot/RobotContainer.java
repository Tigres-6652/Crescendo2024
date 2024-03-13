package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Command.ArmCommand;
import frc.robot.Command.DriveCommand;
import frc.robot.Command.IntakeCommand;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;

public class RobotContainer {

    private final SendableChooser<Command> autoChooser;

//llamar los subsistemas 
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

//declaracion de los controles
  private final Joystick Ctrl = new Joystick(0);


  
  public RobotContainer() {

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("AutoTest", autoChooser);

  configureBindings();
  }

  private void configureBindings() {
//Control del robot
  driveSubsystem.setDefaultCommand(new DriveCommand(driveSubsystem, () -> Ctrl.getRawAxis(1), () -> Ctrl.getRawAxis(4)));

//Control del brazo (movimiento libre)
  //armSubsystem.setDefaultCommand(new ArmCommand(armSubsystem, () -> Ctrl.getRawAxis(1)));

//Control del Intake (seleccion de velocidades)
  new JoystickButton(Ctrl, 1).toggleOnTrue(new IntakeCommand(intakeSubsystem, () -> true, () -> false,  () -> false,  () -> false));
  new JoystickButton(Ctrl, 2).toggleOnTrue(new IntakeCommand(intakeSubsystem, () -> false, () -> true,  () -> false,  () -> false));
  new JoystickButton(Ctrl, 3).toggleOnTrue(new IntakeCommand(intakeSubsystem, () -> false, () -> false,  () -> true,  () -> false));
  new JoystickButton(Ctrl, 4).toggleOnTrue(new IntakeCommand(intakeSubsystem, () -> false, () -> false,  () -> false,  () -> true));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();

  }
}
