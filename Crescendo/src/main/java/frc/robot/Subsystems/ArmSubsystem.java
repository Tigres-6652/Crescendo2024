package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
//Declaracion de los motores
  CANSparkMax RgtMtrArm = new CANSparkMax(6, MotorType.kBrushless);
  CANSparkMax LftMtrArm = new CANSparkMax(5, MotorType.kBrushless);

  ProfiledPIDController PIDMtrArm = new ProfiledPIDController(0.07, 0.0005, 0.00, new TrapezoidProfile.Constraints(80,90));

//Velocidad y Control de los Motores
  public void MtrInvNFllw(double SpdArm) {
    RgtMtrArm.set(-SpdArm);
    LftMtrArm.set(SpdArm);
  }

  public void RgtPstnVrbl (double grados) {
    MtrInvNFllw(-PIDMtrArm.calculate(LftEncPst(), grados));
  }
 

  public double LftEncPst(){
    return -(LftMtrArm.getEncoder().getPosition()/200)*360;
  } 


  public ArmSubsystem() {}

  @Override
  public void periodic() {

SmartDashboard.putNumber("grados eje", LftEncPst());

  }

  // 1 / 200

}