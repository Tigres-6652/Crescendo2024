package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

  TalonFX MAmr1 = new TalonFX (7);
  
  ProfiledPIDController pidController =new ProfiledPIDController
  (0.17, 0.17, 0.0025, new TrapezoidProfile.Constraints(80, 90));

  public void PosicionVariable (double Grado1) {
    var signal= MAmr1.getPosition();
    double FxEnc1 = signal.getValue();
    MAmr1.set(pidController.calculate(FxEnc1, Grado1));
    SmartDashboard.putNumber("Posicion Abajo", FxEnc1);
  }

  public void Botones (boolean Boton_0, boolean Boton_90){

    if((Boton_0)){
      PosicionVariable(0);
    }else if (Boton_90){
      PosicionVariable(90);
    }else{
      stop();
    }
  }

  public void stop(){
    MAmr1.set(0);
  }

  public ArmSubsystem() {}

  @Override
  public void periodic() {}

}
