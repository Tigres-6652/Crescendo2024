// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Command.ArmCommand;
import frc.robot.Command.DriveCommand;
import frc.robot.Command.IntakeCommand;
import frc.robot.Command.Autonomos.Salir;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;

public class RobotContainer {

  public final static DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  private final Joystick XboxController2 = new Joystick(1);
  private final Joystick XboxController = new Joystick(0);
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
  driveSubsystem.setDefaultCommand(new DriveCommand(driveSubsystem, () -> - XboxController.getRawAxis(1), () -> XboxController.getRawAxis(4)));
  
    new JoystickButton(XboxController2, 1).toggleOnTrue(new IntakeCommand(intakeSubsystem, () -> -0.5));
    new JoystickButton(XboxController2, 2).toggleOnTrue(new IntakeCommand(intakeSubsystem, () -> -1.0));
    new JoystickButton(XboxController2, 3).toggleOnTrue(new IntakeCommand(intakeSubsystem, () -> -0.8));
    new JoystickButton(XboxController2, 3).toggleOnTrue(new IntakeCommand(intakeSubsystem, () -> 0.5));

     new JoystickButton(XboxController2, 6).toggleOnTrue(new ArmCommand(armSubsystem, () -> false, ()-> false,() -> XboxController2.getRawAxis(1) * .5, ()  -> false));
     //new JoystickButton(XboxController2, 1).toggleOnTrue(new ArmCommand(armSubsystem, () -> false, ()-> true,() -> XboxController2.getRawAxis(5) * .4, ()  -> false));
     //new JoystickButton(XboxController2, 4).toggleOnTrue(new ArmCommand(armSubsystem, () -> true, ()-> false,() -> XboxController2.getRawAxis(5) * .4, ()  -> false));

  }

  public Command getAutonomousCommand() {
    return new Salir();
  }
}