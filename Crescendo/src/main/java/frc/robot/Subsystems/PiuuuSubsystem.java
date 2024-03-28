package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PiuuuSubsystem extends SubsystemBase {
//Declaracion de Motores del Intake
  WPI_VictorSPX MtrIntk = new WPI_VictorSPX(5);

  TalonFX MtrShtUp = new TalonFX(0);
  TalonFX MtrShtDwn = new TalonFX(0);

//Velocidad del Motor del intake
  public void MtrItkVel(double IktVel) {
    MtrIntk.set(IktVel);
  }

  public void MtrShtVel(double ShtVel) {
    MtrShtUp.set(ShtVel);
    MtrShtDwn.set(-ShtVel*.63);
  }
  public PiuuuSubsystem() {}

  @Override
  public void periodic() {}

}