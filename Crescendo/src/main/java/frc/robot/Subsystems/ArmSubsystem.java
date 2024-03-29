package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  // Declaracion de los motores
  CANSparkMax RgtMtrArm = new CANSparkMax(6, MotorType.kBrushless);
  CANSparkMax LftMtrArm = new CANSparkMax(5, MotorType.kBrushless);

  ProfiledPIDController PIDMtrArm = new ProfiledPIDController(0.04, 0.0005, 0.00,
      new TrapezoidProfile.Constraints(80, 90));

  Joystick joystickreset = new Joystick(4);

  DutyCycleEncoder m_Encoder = new DutyCycleEncoder(0);

  // Velocidad y Control de los Motores
  public void MtrInvNFllw(double SpdArm) {

    if (PosicionEjeGrados() > 80 && SpdArm > 0) {
      RgtMtrArm.set(-SpdArm);
      LftMtrArm.set(SpdArm);
    } else if (PosicionEjeGrados() < 80 && PosicionEjeGrados() > 5) {

      RgtMtrArm.set(-SpdArm);
      LftMtrArm.set(SpdArm);
    } else if (PosicionEjeGrados() < 5 && SpdArm < 0) {
      RgtMtrArm.set(-SpdArm);
      LftMtrArm.set(SpdArm);
    } else {
      RgtMtrArm.set(-0);
      LftMtrArm.set(0);
    }
  }

  public void RgtPstnVrbl(double grados) {
    MtrInvNFllw(-PIDMtrArm.calculate(PosicionEjeGrados(), grados));
  }

  public double PosicionEjeGrados() {
    return (m_Encoder.getDistance() * 360) - 80;
  }

  public ArmSubsystem() {
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("grados eje", PosicionEjeGrados());

    SmartDashboard.putNumber("distance encoder", m_Encoder.getDistance());

    if (joystickreset.getRawButton(1)) {

      m_Encoder.reset();

    }

  }

  // 1 / 200

}