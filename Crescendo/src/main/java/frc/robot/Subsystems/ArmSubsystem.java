package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

  TalonFX MAmr1 = new TalonFX (7);
  

ProfiledPIDController pidController =new ProfiledPIDController(0.19, 0.05, 0.0020, new TrapezoidProfile.Constraints(80, 90));

  public void TalonFXConfigs () {

    //MAmr1.getConfigurator().apply(new TalonFXConfiguration());

    MAmr1.setPosition(0);

  }
  public void control (){

var signal= MAmr1.getPosition();

double posicion = signal.getValue();

SmartDashboard.putNumber("Posicion mot", posicion);

MAmr1.set(pidController.calculate(posicion,15.3333));

  }

  public void stop(){
    MAmr1.set(0);
  }

  public ArmSubsystem() {}

  @Override
  public void periodic() {}
}
