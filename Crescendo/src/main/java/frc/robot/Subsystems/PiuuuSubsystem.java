package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PiuuuSubsystem extends SubsystemBase {

  TalonFX MtrShtUp = new TalonFX(9);
  TalonFX MtrShtDwn = new TalonFX(8);

  OpenLoopRampsConfigs m_ramp = new OpenLoopRampsConfigs();
  VelocityVoltage m_voltage = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
  VelocityVoltage m_voltage2 = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);

  VelocityTorqueCurrentFOC m_torque = new VelocityTorqueCurrentFOC(0, 0, 0, 1, false, false, false);

  NeutralOut m_brake = new NeutralOut();
  
public void configPID(){
  TalonFXConfiguration config = new TalonFXConfiguration();
  config.Slot0.kP=0.003;
  config.Slot0.kI=0.0;
  config.Slot0.kD=0.005;
  config.Slot0.kV=0.12;
  config.Voltage.PeakForwardVoltage=8;
  config.Voltage.PeakReverseVoltage=-8;

  config.TorqueCurrent.PeakForwardTorqueCurrent=40;
  config.TorqueCurrent.PeakReverseTorqueCurrent=-40;


  config.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod=3;
  config.ClosedLoopRamps.TorqueClosedLoopRampPeriod=3;
  config.ClosedLoopRamps.VoltageClosedLoopRampPeriod=3
  ;
  MtrShtDwn.getConfigurator().apply(config);
  MtrShtUp.getConfigurator().apply(config);

}

  public void MtrShtVel(double ShtVel) {

    MtrShtUp.set(ShtVel);
    MtrShtDwn.set(ShtVel*0.63);
  }

  public void ShootRPM(double rpms){

    MtrShtDwn.setControl(m_voltage.withVelocity(rpms).withAcceleration(0.2));
    MtrShtUp.setControl(m_voltage2.withVelocity(rpms).withAcceleration(0.2));


  }

  public void motoroff(){

    MtrShtDwn.setControl(m_brake);
    MtrShtUp.setControl(m_brake);

  }


  public PiuuuSubsystem() {}

  @Override
  public void periodic() {

    SmartDashboard.putNumber("RPM down", MtrShtDwn.getVelocity().getValue());
    SmartDashboard.putNumber("RPM up", MtrShtUp.getVelocity().getValue());


  }

}