package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {


  CANSparkMax RgtMtrArm = new CANSparkMax(7, MotorType.kBrushless);
  CANSparkMax LftMtrArm = new CANSparkMax(8, MotorType.kBrushless);

  ProfiledPIDController PIDRgtMtrArm = new ProfiledPIDController(0.05, 0.0005, 0.00, new TrapezoidProfile.Constraints(80,90));
  ProfiledPIDController PIDLftMtrArm = new ProfiledPIDController(0.05, 0.0005, 0.00, new TrapezoidProfile.Constraints(80,90));

  public void RgtPstnVrbl (double Grado1) {
    var signal= RgtMtrArm.getEncoder();
    double RgtNeoEnc = signal.getPosition();
    RgtMtrArm.set(PIDRgtMtrArm.calculate(RgtNeoEnc, Grado1));
    SmartDashboard.putNumber("Posicion Abajo", RgtNeoEnc);
  }

  public void LftPstnVrbl (double Grado2) {
    var signal= LftMtrArm.getEncoder();
    double LftNeoEnc = signal.getPosition();
    LftMtrArm.set(PIDLftMtrArm.calculate(LftNeoEnc, Grado2));
    SmartDashboard.putNumber("Posicion Abajo", LftNeoEnc);
  }


/*
  TalonFX MAmr1 = new TalonFX (7);
  
  ProfiledPIDController pidController =new ProfiledPIDController
  (0.17, 0.17, 0.0025, new TrapezoidProfile.Constraints(80, 90));

  public void PosicionVariable (double Grado1) {
    var signal= MAmr1.getPosition();
    double FxEnc1 = signal.getValue();
    MAmr1.set(pidController.calculate(FxEnc1, Grado1));
    SmartDashboard.putNumber("Posicion Abajo", FxEnc1);
  } 
*/

  public void stop (double speed){
    RgtMtrArm.set(0);
  }

  public void Botones (boolean Boton_0, boolean Boton_90){

    if((Boton_0)){
      RgtPstnVrbl(0);

    }else if (Boton_90){
      RgtPstnVrbl(90);

    }else{
      stop(0);
    }
  }

  public ArmSubsystem() {}

  @Override
  public void periodic() {}
}
