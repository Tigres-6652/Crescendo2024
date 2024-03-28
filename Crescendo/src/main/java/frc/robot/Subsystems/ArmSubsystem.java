package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
//Declaracion de los motores
  CANSparkMax RgtMtrArm = new CANSparkMax(7, MotorType.kBrushless);
  CANSparkMax LftMtrArm = new CANSparkMax(6, MotorType.kBrushless);

  Encoder enc = new Encoder(null, null);


//Velocidad y Control de los Motores
  public void MtrInvNFllw(double SpdArm) {
    RgtMtrArm.set(SpdArm);
    LftMtrArm.set(-SpdArm);
  }
  
  public ArmSubsystem() {}

  @Override
  public void periodic() {}

}