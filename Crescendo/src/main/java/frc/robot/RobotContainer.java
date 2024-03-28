package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Command.ArmCommand;
import frc.robot.Command.DriveCommand;
import frc.robot.Command.PiuuuCommand;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.PiuuuSubsystem;

public class RobotContainer {
//llamar los subsistemas 
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final PiuuuSubsystem piuuuSubsystem = new PiuuuSubsystem();

//declaracion de los controles
  private final Joystick FirstD = new Joystick(0);
  private final Joystick SecondD = new Joystick(1);

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {


    
  configureBindings();
//building the auto chooser for pathplanner

  autoChooser = AutoBuilder.buildAutoChooser();
  SmartDashboard.putData("AutoChooser", autoChooser);
  }

  private void configureBindings() {
//Control del robot
  driveSubsystem.setDefaultCommand(new DriveCommand(driveSubsystem, () -> FirstD.getRawAxis(1), () -> FirstD.getRawAxis(4)));

//Control del brazo (movimiento libre)
  armSubsystem.setDefaultCommand(new ArmCommand(armSubsystem, () -> SecondD.getRawAxis(1)));

//Control del Intake (seleccion de velocidades)
  new JoystickButton(SecondD, 1).toggleOnTrue(new PiuuuCommand(piuuuSubsystem, () -> true, () -> false,  () -> false,  () -> false, () -> false, () -> false));
  new JoystickButton(SecondD, 2).toggleOnTrue(new PiuuuCommand(piuuuSubsystem, () -> false, () -> true,  () -> false,  () -> false, () -> false, () -> false));
  new JoystickButton(SecondD, 3).toggleOnTrue(new PiuuuCommand(piuuuSubsystem, () -> false, () -> false,  () -> true,  () -> false, () -> false, () -> false));
  new JoystickButton(SecondD, 4).toggleOnTrue(new PiuuuCommand(piuuuSubsystem, () -> false, () -> false,  () -> false,  () -> true, () -> false, () -> false));
                                                                                                                                                      //Disp1       Disp2
  new JoystickButton(SecondD, 5).toggleOnTrue(new PiuuuCommand(piuuuSubsystem, () -> false, () -> false,  () -> false,  () -> false, () -> true, () -> false));
  new JoystickButton(SecondD, 6).toggleOnTrue(new PiuuuCommand(piuuuSubsystem, () -> false, () -> false,  () -> false,  () -> false, () -> false, () -> true));

}

  public Command getAutonomousCommand() {

    return autoChooser.getSelected();
  }
}