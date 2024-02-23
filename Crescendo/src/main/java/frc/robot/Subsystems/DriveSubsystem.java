package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/////////////////////////////////////////////////////////////////////////////////////////////////////

public class DriveSubsystem extends SubsystemBase {
//Declaracione de los motores derecchos
  static WPI_TalonSRX RMtrEnc = new WPI_TalonSRX(1);
  WPI_TalonSRX RMtrFllw = new WPI_TalonSRX(2);

//Declaracion de los motores izquierdos
  static WPI_TalonSRX LMtrEnc = new WPI_TalonSRX(3);
  WPI_TalonSRX LMtrFllw = new WPI_TalonSRX(4);

//Declaracion para el control diferencial de los motores que estan en el chasis
  DifferentialDrive Chasis = new DifferentialDrive(LMtrEnc, RMtrEnc);

/////////////////////////////////////////////////////////////////////////////////////////////////////

  @Override
  public void periodic() {
    SmartDashboard.putNumber("RgtEnc", RgtEnc());
    SmartDashboard.putNumber("LftEnc", LftEnc());
  }

//Metodo para controla el chasis
  public void Arcade_Drive (double Speed, double Spin){
    Chasis.arcadeDrive(Speed, Spin);
  }

//Metodo para invertir los motores izquierdos
  public static void Inverte () {
    RMtrEnc.setInverted(false);
    LMtrEnc.setInverted(true);
  }

//Metodo para seguir los motores principales
  public static void FollowMotor (){
    RMtrEnc.follow(RMtrEnc);
    LMtrEnc.follow(LMtrEnc);
  }

//Encoder Derecho
  public double RgtEnc () {
    return (RMtrEnc.getSelectedSensorPosition() / 4096 * Math.PI * 6 * 10 * 2.54 / 100);

  }

//Encoder izquierdo
  public double LftEnc () {
    return (LMtrEnc.getSelectedSensorPosition() / 4096 * Math.PI * 6 * 10 * 2.54 / 100);

  }

//De Pulsaciones a Distancia
  public double DtncPlss (double DstncMTS) {
    double pulses = (((((DstncMTS * 100) / 2.54) / 6) / Math.PI) * 4096);

    return pulses;

  }
}