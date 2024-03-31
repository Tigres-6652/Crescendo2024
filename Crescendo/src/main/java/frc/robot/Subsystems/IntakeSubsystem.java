// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
//Declaracion de Motores del Intake
  WPI_VictorSPX MtrIntk = new WPI_VictorSPX(7);
  
  PowerDistribution pdp = new PowerDistribution();
  public IntakeSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("CIM", pdp.getCurrent(5));
        SmartDashboard.putNumber("shoot1", pdp.getCurrent(3));
    SmartDashboard.putNumber("shoot2", pdp.getCurrent(15));

  }

//Velocidad del Motor del intake
public void MtrItkVel(double IktVel) {
  MtrIntk.set(IktVel);
}

}
