package frc.robot.Subsystems;

import java.nio.file.Path;
import java.util.List;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.proto.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

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
  DifferentialDriveOdometry m_odometry;
  DifferentialDriveKinematics m_kinematics;
  Field2d m_Field2d;
  AHRS Navx = new AHRS(SPI.Port.kMXP);
Trajectory traye;
  Rotation2d rot = new Rotation2d();
  
  //List<PathPlannerAuto> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile("si");
  //private Pose2d posee = PathPlannerAuto.getStaringPoseFromAutoFile("si");



//=================================================================================================================\\
  public DriveSubsystem() {   
  //configtalon();
  //Smartdashboard();
  //RgtEnc();
  //RgtVel();
  //LftEnc();
  //LftVel(); 

  m_Field2d = new Field2d();
  m_kinematics = new DifferentialDriveKinematics(0.3556);

       /*  AutoBuilder.configureRamsete(
        this::getPose,
        this::resetPose,
        this::getCurrentSpeeds,
        this::driveChassisSpeeds,
        new ReplanningConfig(),
        () -> {

        var alliance = DriverStation.getAlliance();
         if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      this
    );*/


  m_odometry = new DifferentialDriveOdometry(getRotation2d(), LftEnc(), RgtEnc());

    AutoBuilder.configureRamsete(        
        this::getPose,
        this::resetPose,
        this::getCurrentSpeeds,
        this::driveChassisSpeeds,
        
        new ReplanningConfig(),
        () -> {
        configtalon();
        //Reset();
        var alliance = DriverStation.getAlliance();
         if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      this);
  }

  @Override
  public void periodic() {
    
    m_odometry.update(getRotation2d(), LftEnc(), RgtEnc());
    Smartdashboard();

    m_Field2d.setRobotPose(getPose());

    SmartDashboard.putString("pose", getPose().toString());
  }

//==Metodo para controla el chasis=====================
  public void Arcade_Drive(double Speed, double Giro){
    Chasis.arcadeDrive(Speed, Giro);
  }

//==tankdrive==========================================
  public void tanque(double Lft, double Rgt) {
    Chasis.tankDrive(-Lft*0.3, -Rgt*0.3);

   // Chasis.tankDrive(Lft, Rgt);
  }

//==Ecoders============================================
  public double RgtEnc() {
    return (RMtrEnc.getSelectedSensorPosition() / 4096 * Math.PI * 6 * 2.54) / 100;
  }
  public double LftEnc() {
    return (LMtrEnc.getSelectedSensorPosition() / 4096 * Math.PI * 6 * 2.54) / 100;
  }

//==Velocidad del los motores===========================
  public double RgtVel() {
    return (RMtrEnc.getSelectedSensorVelocity() / 4096 * Math.PI * 6 * 2.54) / 10;
  }
  public double LftVel() {
    return (LMtrEnc.getSelectedSensorVelocity() / 4096 * Math.PI * 6 * 2.54) / 10;
  }

//Lectura de rotacion Navx
  public Rotation2d getRotation2d() {
    return Navx.getRotation2d();
  }

//==ResetEncoders=======================================
  public void Reset() {
    RMtrEnc.setSelectedSensorPosition(0);
    LMtrEnc.setSelectedSensorPosition(0);
    Navx.reset();
  }

//==GetPose=============================================
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

//==ResetPose===========================================
  public void resetPose(Pose2d pose){
  SmartDashboard.putString("reset", pose.toString());


    //pose = PathPlannerAuto.getStaringPoseFromAutoFile("si");
  
    m_odometry.resetPosition(Navx.getRotation2d(), LftEnc(), RgtEnc(), pose);
        m_odometry = new DifferentialDriveOdometry(Navx.getRotation2d(), LftEnc(), RgtEnc(),pose);

    //pose = new Pose2d(2,2, getRotation2d());
    //    m_odometry.resetPosition(pose.getRotation(), 0, 0, getPose());
    
  }

  public void resetCordenadas() {

    m_odometry = new DifferentialDriveOdometry(Navx.getRotation2d(), 0, 0);
  }

//==GetSpeeds===========================================
  public ChassisSpeeds getCurrentSpeeds() {
    return m_kinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(LftVel(), RgtVel()));
  }

//==SetSpeeds===========================================
  public void driveChassisSpeeds(ChassisSpeeds speeds){
    DifferentialDriveWheelSpeeds diffSpeeds = m_kinematics.toWheelSpeeds(speeds);

    SmartDashboard.putNumber("Vlocidad left", diffSpeeds.leftMetersPerSecond);
    SmartDashboard.putNumber("Vlocidad right", diffSpeeds.rightMetersPerSecond);

    tanque(diffSpeeds.leftMetersPerSecond, diffSpeeds.rightMetersPerSecond); 
  } 

/* public void driveChassisSpeeds(ChassisSpeeds speed) {
    double linearSpeed = speed.vxMetersPerSecond;
    double rotSpeed = speed.omegaRadiansPerSecond;
    
    Arcade_Drive(linearSpeed, rotSpeed);
  } */

//==SmartDashboard======================================
  public void Smartdashboard() {
    SmartDashboard.putNumber("Encoder derecho", RgtEnc());
    SmartDashboard.putNumber("Velocidad derecho", RgtVel());

    SmartDashboard.putNumber("Encoder izquuierdo", LftEnc());
    SmartDashboard.putNumber("Velocidad izquierdo", LftVel());
    //Smartdashboard.putnum
  }

//==Configuracion de los motores y PID==================
  public void configtalon() {


    
    RMtrFllw.follow(RMtrEnc);
    LMtrFllw.follow(LMtrEnc);
    
    LMtrEnc.setInverted(true);
    RMtrEnc.setInverted(false);
    
    RMtrFllw.setInverted(InvertType.FollowMaster);
    LMtrFllw.setInverted(InvertType.FollowMaster);
    
    RMtrEnc.setSensorPhase(true);
    LMtrEnc.setSensorPhase(true);
  }
}