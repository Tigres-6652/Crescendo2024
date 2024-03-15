package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
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
  static WPI_TalonSRX LMtrEnc = new WPI_TalonSRX(3);
  static WPI_TalonSRX LMtrFllw = new WPI_TalonSRX(4);

//Declaracion para el control diferencial de los motores que estan en el chasis
  DifferentialDrive Chasis = new DifferentialDrive(LMtrEnc, RMtrEnc);

//NAVX y Odometria 
  AHRS Navx = new AHRS(SPI.Port.kMXP);
  DifferentialDriveOdometry m_odometry;
  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(20.5));
//=================================================================================================================\\

//Configuracion de los motores y navx
//Seguimiento e inversion de los motores e inicialisacion de la odometria
  public DriveSubsystem() {

    
    m_odometry = new DifferentialDriveOdometry(getRotation2d(), LftEnc(), RgtEnc());

    AutoBuilder.configureRamsete(
      this::getPose,
      this::resetOdometry,
      this::getSpeeds,
      this::setChassisSpeeds,
      new ReplanningConfig(),
      () -> {

      var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
        return alliance.get() == DriverStation.Alliance.Red;
      }
      return false;
    },
    this
  );
}

  public void setChassisSpeeds(ChassisSpeeds speed) {
    double linearSpeed = speed.vxMetersPerSecond;
    double rotSpeed = speed.omegaRadiansPerSecond;
    
    Arcade_Drive(linearSpeed, rotSpeed);
  }

  public ChassisSpeeds getSpeeds(){
      DifferentialDriveWheelSpeeds wheelSpeeds  =  new DifferentialDriveWheelSpeeds(LftVel(), RgtVel());
      return kinematics.toChassisSpeeds(wheelSpeeds);
    }

    //Reseteo de la posicion
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(getRotation2d(), LftEnc(), RgtEnc(), pose);
  }
  
    //Posicion en metros
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

//===============================================================================================
  public void configtalon() {
    RMtrEnc.configFactoryDefault();
    RMtrFllw.configFactoryDefault();
    LMtrEnc.configFactoryDefault();
    LMtrFllw.configFactoryDefault();

    RMtrFllw.follow(RMtrEnc);
    LMtrFllw.follow(LMtrEnc);

    RMtrEnc.setInverted(true);
    LMtrEnc.setInverted(false);

    RMtrEnc.setInverted(InvertType.FollowMaster);
    LMtrEnc.setInverted(InvertType.FollowMaster);

    RMtrEnc.setSensorPhase(true);
    LMtrEnc.setSensorPhase(true);

    RMtrEnc.configNominalOutputForward(0, 30);
    RMtrEnc.configNominalOutputReverse(0, 30);
    RMtrEnc.configPeakOutputForward(1, 30);
    RMtrEnc.configPeakOutputReverse(-1, 30);

    RMtrEnc.config_kF(0, 0, 30);
    RMtrEnc.config_kP(0, 0, 30);
    RMtrEnc.config_kI(0, 0, 30);
    RMtrEnc.config_kD(0, 0, 30);

    LMtrEnc.configNominalOutputForward(0, 30);
    LMtrEnc.configNominalOutputReverse(0, 30);
    LMtrEnc.configPeakOutputForward(1, 30);
    LMtrEnc.configPeakOutputReverse(-1, 30);

    LMtrEnc.config_kF(0, 0, 30);
    LMtrEnc.config_kP(0, 0, 30);
    LMtrEnc.config_kI(0, 0, 30);
    LMtrEnc.config_kD(0, 0, 30);
  }


  @Override
  public void periodic() {
    m_odometry.update(Navx.getRotation2d(), LftEnc(), RgtEnc()); 
    Chasis.feed();
  }

//Metodo para controla el chasis
  public void Arcade_Drive(double Speed, double Giro){
    Chasis.arcadeDrive(Speed, Giro);

  }

//Metodos para los encoders 
//Encoders
  public double Encoders() {
    return (RgtEnc() + LftEnc() / 2);
  }

//Encoder derecho
  public double RgtEnc() {
    return (RMtrEnc.getSelectedSensorPosition() / 4096 * Math.PI * 6 * 2.54) / 100;
  }

//Encoder Izquierdo
  public double LftEnc() {
    return (LMtrEnc.getSelectedSensorPosition() / 4096 * Math.PI * 6 * 2.54) / 100;
  }

  //Encoder derecho
  public double RgtVel() {
    return (RMtrEnc.getSelectedSensorVelocity() / 4096 * Math.PI * 6 * 2.54) / 100;
  }

//Encoder Izquierdo
  public double LftVel() {
    return (LMtrEnc.getSelectedSensorVelocity() / 4096 * Math.PI * 6 * 2.54) / 100;
  }

//Reseteo de los encoders
  public void resetEncoders() {
    RMtrEnc.setSelectedSensorPosition(0);
    LMtrEnc.setSelectedSensorPosition(0);
  }

//SmartDashboard de los enocders
  public void SDEncders () {
    SmartDashboard.putNumber("Distancia encoder derecho", RgtEnc());
    SmartDashboard.putNumber("Distancia encoder izquierdo", LftEnc());
    
    SmartDashboard.putNumber("angle", Navx.getAngle());
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



//Resetiado de la navx
  public void zeroHeading() {
    Navx.reset();
  }

//Reseteo de los sensores
  public void Reset() {
    zeroHeading();
    resetEncoders();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(LftEnc(), RgtEnc());
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    LMtrEnc.setVoltage(leftVolts);
    RMtrEnc.setVoltage(rightVolts);
    Chasis.feed();
  }

  public void setMaxOutput(double maxOutput) {
    Chasis.setMaxOutput(maxOutput);
  }
}