package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PiuuuSubsystem extends SubsystemBase {

  TalonFX MtrShtUp = new TalonFX(9);
  TalonFX MtrShtDwn = new TalonFX(8);



  public void MtrShtVel(double ShtVel) {
    MtrShtUp.set(ShtVel);
    MtrShtDwn.set(ShtVel*0.63);
  }
  public PiuuuSubsystem() {}

  @Override
  public void periodic() {}

}