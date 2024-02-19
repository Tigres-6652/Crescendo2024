// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Command.ArmCommand;
import frc.robot.Command.DriveCommand;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.DriveSubsystem;

public class RobotContainer {

  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();

  private final Joystick XboxController = new Joystick(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
  driveSubsystem.setDefaultCommand(new DriveCommand(driveSubsystem, () -> XboxController.getRawAxis(1), () -> XboxController.getRawAxis(4)));

  new JoystickButton(XboxController, 2).toggleOnTrue(new ArmCommand(armSubsystem, () -> true, () -> false, () -> false, () -> false, () -> false));
  new JoystickButton(XboxController, 1).toggleOnTrue(new ArmCommand(armSubsystem, () -> false, () -> true, () -> false, () -> false, () -> false));
  new JoystickButton(XboxController, 3).toggleOnTrue(new ArmCommand(armSubsystem, () -> false, () -> false, () -> true, () -> false, () -> false));
  new JoystickButton(XboxController, 4).toggleOnTrue(new ArmCommand(armSubsystem, () -> false, () -> false, () -> false, () -> true, () -> false));
  new JoystickButton(XboxController, 5).toggleOnTrue(new ArmCommand(armSubsystem, () -> false, () -> false, () -> false, () -> false, () -> true));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
