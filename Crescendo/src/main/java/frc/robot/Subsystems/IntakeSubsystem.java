// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

//Motor del intake
 WPI_VictorSPX MtrIntk = new WPI_VictorSPX(9);

//Limit swich del intake 
 DigitalInput LmtIntk = new DigitalInput(9);

//metodo para contorlar el motor del intake
  public void MotorIntake (boolean boton1){
    
    if (boton1) {
      MtrIntk.set(0.5);

    }else{
      MtrIntk.set(0);

    }
  }

  public IntakeSubsystem() {}

  @Override
  public void periodic() {
  }
}
