package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
//Declaracione de los motores derecchos
  static WPI_TalonSRX RMtrEnc = new WPI_TalonSRX(1);
  static WPI_TalonSRX RMtrFllw = new WPI_TalonSRX(2);

//Declaracion de los motores izquierdos
  static WPI_TalonSRX LMtrEnc = new WPI_TalonSRX(4);
  static WPI_TalonSRX LMtrFllw = new WPI_TalonSRX(3);

//Declaracion para el control diferencial de los motores que estan en el chasis
  DifferentialDrive Chasis = new DifferentialDrive(LMtrEnc, RMtrEnc);
  
  public void ChasisSub() {}

//Metodo para controla el chasis
  public void Arcade_Drive(double Speed, double Giro){
    Chasis.arcadeDrive(Speed, Giro);

  }

//Metodo para invertir los motores izquierdos
  public static void Inverte() {
    RMtrEnc.setInverted(false);
    LMtrEnc.setInverted(true);

  }

//Metodo para seguir los motores principales
  public static void FollowMotor(){
    RMtrFllw.follow(RMtrEnc);
    LMtrFllw.follow(LMtrEnc);
    
  }

//================================================================================================================//
 
  public DriveSubsystem() {}

  @Override
  public void periodic() {}

}