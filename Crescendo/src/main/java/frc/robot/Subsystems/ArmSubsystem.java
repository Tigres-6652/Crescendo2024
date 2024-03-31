package frc.robot.Subsystems;

import org.opencv.core.Mat;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class ArmSubsystem extends SubsystemBase {
  // Declaracion de los motores
  CANSparkMax RgtMtrArm = new CANSparkMax(6, MotorType.kBrushless);
  CANSparkMax LftMtrArm = new CANSparkMax(5, MotorType.kBrushless);

  ProfiledPIDController PIDMtrArm = new ProfiledPIDController(0.1, 0.0, 0.00,
      new TrapezoidProfile.Constraints(40, 20));


  DutyCycleEncoder m_Encoder = new DutyCycleEncoder(0);
  String lime = "limelight-limee";
      double angulo;
      double fijo = 14;
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
    if(distance()>1){
    angulo = 36+fijo-(Math.toDegrees(Math.tanh(1.36/distance())))+fijo;

    }
    SmartDashboard.putNumber("Distancia limelight", distance());

    SmartDashboard.putNumber("grados eje", PosicionEjeGrados());

    SmartDashboard.putNumber("distance encoder", m_Encoder.getDistance());
      SmartDashboard.putNumber("angulocalculado", angulo);


  }

    public void anguloVariable(){




          RgtPstnVrbl(angulo);

    }


    public double distance(){
    
    double targetOffsetAngle_Vertical = LimelightHelpers.getTY(lime);
    // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = 30.0; 

    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = 10.23; 

    // distance from the target to the floor
    double goalHeightInches = 53.88; 

    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    //calculate distance
    double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
    double distanceFromLimelightToGoalMeters = (distanceFromLimelightToGoalInches*2.54)/100;

    return distanceFromLimelightToGoalMeters;
}


  // 1 / 200

}