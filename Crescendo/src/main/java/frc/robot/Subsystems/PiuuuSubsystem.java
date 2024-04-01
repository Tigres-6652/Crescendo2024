package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class PiuuuSubsystem extends SubsystemBase {

  TalonFX MtrShtUp = new TalonFX(9);
  TalonFX MtrShtDwn = new TalonFX(8);

  OpenLoopRampsConfigs m_ramp = new OpenLoopRampsConfigs();
  VelocityVoltage m_voltage = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
  VelocityVoltage m_voltage2 = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);

  

  NeutralOut m_brake = new NeutralOut();
  String lime = "limelight-limee";

public void configPID(){
  TalonFXConfiguration config = new TalonFXConfiguration();
  config.Slot0.kP=2.2;
  config.Slot0.kI=0.0;
  config.Slot0.kD=0.00;
  config.Slot0.kV=0.12;
  config.Voltage.PeakForwardVoltage=8;
  config.Voltage.PeakReverseVoltage=-8;

  CurrentLimitsConfigs m_currentLimits = new CurrentLimitsConfigs();

   m_currentLimits.SupplyCurrentLimit = 30; // Limit to 1 amps
   m_currentLimits.SupplyCurrentThreshold = 30; // If we exceed 4 amps
   m_currentLimits.SupplyTimeThreshold = 0.5; // For at least 1 second
   m_currentLimits.SupplyCurrentLimitEnable = true; // And enable it

   m_currentLimits.StatorCurrentLimit = 50; // Limit stator to 20 amps
   m_currentLimits.StatorCurrentLimitEnable = true; // And enable it
   
   config.CurrentLimits = m_currentLimits;


  config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod=3;
  config.OpenLoopRamps.TorqueOpenLoopRampPeriod=3;
  config.OpenLoopRamps.VoltageOpenLoopRampPeriod=3;

  //config.OpenLoopRamps.;
  
  MtrShtDwn.getConfigurator().apply(config);
  MtrShtUp.getConfigurator().apply(config);

}

  public void MtrShtVel(double ShtVel) {


    MtrShtUp.set(ShtVel*.80);
    MtrShtDwn.set(ShtVel*0.63);
  }

  public void ShootRPM(double rps){

    double rpss=rps+(distance()*5);

    MtrShtDwn.setControl(m_voltage.withVelocity(rpss*0.63));
    MtrShtUp.setControl(m_voltage2.withVelocity(rpss));

   //MtrShtDwn.setControl(m_torque.withVelocity(rpms));

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


}