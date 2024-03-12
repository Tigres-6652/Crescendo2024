package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
//Declaracion de Motores del Intake
  WPI_VictorSPX MtrIntk = new WPI_VictorSPX(5);

//Velocidad del Motor del intake
  public void MtrItkVel(double IktVel) {
    MtrIntk.set(IktVel);
  }

  public IntakeSubsystem() {}

  @Override
  public void periodic() {}

}