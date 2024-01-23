package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

  TalonFX MAmr1 = new TalonFX (5);
  
  PositionVoltage MVolPos1 = new PositionVoltage(0 , 0, true, 0, 0, false, false, false);

  PositionTorqueCurrentFOC MTorPos1 = new PositionTorqueCurrentFOC(0 ,0, 0,1,false,false,false);

  NeutralOut Mbrakem = new NeutralOut();

  public void TalonFXConfigs () {

    TalonFXConfiguration configs = new TalonFXConfiguration();

    configs.Slot0.kP = 2.4; // An error of 0.5 rotations results in 1.2 volts output
    configs.Slot0.kD = 0.1; // A change of 1 rotation per second results in 0.1 volts output
    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = 8;
    configs.Voltage.PeakReverseVoltage = -8;
    
    configs.Slot1.kP = 40; // An error of 1 rotations results in 40 amps output
    configs.Slot1.kD = 2; // A change of 1 rotation per second results in 2 amps output
    // Peak output of 130 amps
    configs.TorqueCurrent.PeakForwardTorqueCurrent = 130;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = 130;

  }

  public void control (){
    MAmr1.setControl(MVolPos1.withPosition(0));
    
  }

  public ArmSubsystem() {}

  @Override
  public void periodic() {}
}
