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
  // llamar los subsistemas
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final PiuuuSubsystem piuuuSubsystem = new PiuuuSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  // declaracion de los controles
  private final Joystick FirstD = new Joystick(0);
  private final Joystick SecondD = new Joystick(1);

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {

    NamedCommands.registerCommand("intakeSTOP", new IntakeCommand(intakeSubsystem, () -> false, () -> false));
    NamedCommands.registerCommand("intakeRUN", new IntakeCommand(intakeSubsystem, () -> true, () -> false));
    NamedCommands.registerCommand("intakedesRUN", new InstantCommand(() -> intakeSubsystem.MtrItkVel(0.6), intakeSubsystem));

    NamedCommands.registerCommand("piuuuRUN", new PiuuuCommand(piuuuSubsystem, () -> false, () -> true, () -> false));
    NamedCommands.registerCommand("piuuuSTOP", new PiuuuCommand(piuuuSubsystem, () -> false, () -> false, () -> false));

    NamedCommands.registerCommand("AcomodoRUN", new ArmCommand(armSubsystem, () -> 0.0, () -> false, () -> true, () -> false, () -> false, () -> false));
    NamedCommands.registerCommand("ArmRUN", new ArmCommand(armSubsystem, () -> 0.0, () -> false, () -> true, () -> false, () -> false, () -> false));
    NamedCommands.registerCommand("ArmWaitUp", new ArmCommand(armSubsystem, () -> 0.0, () -> false, () -> false, () -> true, () -> false, () -> false));
    NamedCommands.registerCommand("ArmWait", new ArmCommand(armSubsystem, () -> 0.0, () -> false, () -> false, () -> false, () -> false, () -> true));

    NamedCommands.registerCommand("aimbot", new InstantCommand(() -> driveSubsystem.AimAndDist(0), driveSubsystem));

    configureBindings();
    // building the auto chooser for pathplanner

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("AutoChooser", autoChooser);
  }

  private void configureBindings() {
    // Control del robot
    driveSubsystem.setDefaultCommand(new DriveCommand(driveSubsystem,
        () -> (((FirstD.getRawAxis(3) * .40) - (FirstD.getRawAxis(1) * .6)) - (FirstD.getRawAxis(2) * .4)),
        () -> FirstD.getRawAxis(4) * .8, () -> FirstD.getRawButton(1)));

    // Control del Intake (seleccion de velocidades)
    new JoystickButton(SecondD, 3).toggleOnTrue(new IntakeCommand(intakeSubsystem, () -> true, () -> false));
    new JoystickButton(SecondD, 2).toggleOnTrue(new IntakeCommand(intakeSubsystem, () -> false, () -> true));

    // Control del brazo (movimiento libre)

    armSubsystem.setDefaultCommand(new ArmCommand(armSubsystem,
        () -> SecondD.getRawAxis(1), () -> SecondD.getRawButton(4),
       () -> SecondD.getRawButton(1), () -> false/* SecondD.getRawButtonnull) */,
        () -> false, () -> false));

    // Control de shooter
    new POVButton(SecondD, 0).toggleOnTrue(new PiuuuCommand(piuuuSubsystem, () -> false, () -> false, () -> true));
    new POVButton(SecondD, 90).toggleOnTrue(new PiuuuCommand(piuuuSubsystem, () -> true, () -> false, () -> false));
    new POVButton(SecondD, 270).toggleOnTrue(new PiuuuCommand(piuuuSubsystem, () -> false, () -> true, () -> false));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}