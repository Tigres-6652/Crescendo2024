package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  CANSparkMax RgtMtrArm = new CANSparkMax(5, MotorType.kBrushless);
  CANSparkMax LftMtrArm = new CANSparkMax(6, MotorType.kBrushless);
  
  DigitalInput limit = new DigitalInput(0);
 
  double valor;

  double velmaximapid = 0.4;

  ProfiledPIDController PIDLftMtrArm = new ProfiledPIDController(0.03, 0.0005, 0.00,
  new TrapezoidProfile.Constraints(90, 80));

  public void LftPstnVrbl(double Grado2) {
    var signal = LftMtrArm.getEncoder();
    double LftNeoEnc = signal.getPosition();
    double pidcalculo = PIDLftMtrArm.calculate(LftNeoEnc, Grado2);
    SmartDashboard.putNumber("Posicion Abajo", LftNeoEnc);

    if (pidcalculo > velmaximapid) {
      valor = velmaximapid;

    } else if (pidcalculo < -velmaximapid) {
      valor = -velmaximapid;

    } else {
      valor = pidcalculo;

    }

    LftMtrArm.set(-0.06);
    RgtMtrArm.set(0.06);
    SmartDashboard.putNumber("EJE", valor);
  
  }

  public void Configs() {
    LftMtrArm.getEncoder().setPosition(0);
  
  }

  public void stop() {}

  public void manejolibre(double vel) {
      RgtMtrArm.set(-vel);
      LftMtrArm.set(vel);

  }

  public ArmSubsystem() {}

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("chi", limit.get());

    var signal = LftMtrArm.getEncoder();
    double LftNeoEnc = signal.getPosition();

  }
}
