// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  WPI_VictorSPX MtrIntk = new WPI_VictorSPX(7);
  
  DigitalInput Limit = new DigitalInput(0);

  public void MtrItk (double vel){
    MtrIntk.set(vel);
  }

  public void limit (){
    SmartDashboard.putBoolean("Limite", Limit.get());

  }
  public IntakeSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
