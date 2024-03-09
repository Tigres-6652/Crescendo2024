package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
//Motores
//Declaracione de los motores derecchos
  static WPI_TalonSRX RMtrEnc = new WPI_TalonSRX(1);
  static WPI_TalonSRX RMtrFllw = new WPI_TalonSRX(2);

//Declaracion de los motores izquierdos
  static WPI_TalonSRX LMtrEnc = new WPI_TalonSRX(4);
  static WPI_TalonSRX LMtrFllw = new WPI_TalonSRX(3);

//Declaracion para el control diferencial de los motores que estan en el chasis
  DifferentialDrive Chasis = new DifferentialDrive(LMtrEnc, RMtrEnc);

//NAVX y Odometria 
//Declaracion de las navx
  AHRS Navx = new AHRS(SPI.Port.kMXP);
  DifferentialDriveOdometry m_odometry;

//=================================================================================================================//
//Seguimiento e inversion de los motores e inicialisacion de la odometria
  public DriveSubsystem() {
    RMtrEnc.setInverted(false);
    LMtrEnc.setInverted(true);

    RMtrFllw.follow(RMtrEnc);
    LMtrFllw.follow(LMtrEnc);
  
    m_odometry = new DifferentialDriveOdometry(getRotation2d(), LftEnc(), RgtEnc());

  }

//Metodo para controla el chasis
  public void Arcade_Drive(double Speed, double Giro){
    Chasis.arcadeDrive(Speed, Giro);

    SmartDashboard.putNumber("Distancia encoder derecho", RgtEnc());
    SmartDashboard.putNumber("Distancia encoder izquierdo", LftEnc());

    SmartDashboard.putNumber("angle", Navx.getAngle());
  }

//Encoders
//Encoder derecho
  public double RgtEnc() {
    return (RMtrEnc.getSelectedSensorPosition() / 4096 * Math.PI * 6 * 2.54) / 100;
  
  }

//Encoder Izquierdo
  public double LftEnc() {
    return (LMtrEnc.getSelectedSensorPosition() / 4096 * Math.PI * 6 * 2.54) / 100;

  }
//Reseteo de los encoders
  public void resetEncoders() {
    RMtrEnc.setSelectedSensorPosition(0);
    LMtrEnc.setSelectedSensorPosition(0);

  }

//Configuraciones Navx
//Plano 2D
  public Rotation2d getRotation2d() {
    return Navx.getRotation2d();

  }

//Eje x?
  public double getTurnRate() {
    return Navx.getRate();

  }

//Navx pero en grados  xd
  public double getHeading() {
    return Navx.getRotation2d().getDegrees();

  }

//Posicion en metros
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

//Reseteo de la posicion
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(getRotation2d(), LftEnc(), RgtEnc(), pose);

  }

//Resetiado de la navx
  public void zeroHeading() {
    Navx.reset();

  }

//Reseteo de los sensores
  public void Reset() {
    zeroHeading();
    resetEncoders();
  }

  @Override
  public void periodic() {}

}