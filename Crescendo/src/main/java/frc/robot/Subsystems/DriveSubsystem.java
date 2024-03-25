package frc.robot.Subsystems;


import java.net.NetworkInterface;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
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
import edu.wpi.first.math.proto.Trajectory;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;


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

  double xd = LimelightHelpers.getTX("xd");



  //List<PathPlannerAuto> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile("si");
  //private Pose2d posee = PathPlannerAuto.getStaringPoseFromAutoFile("si");

//=================================================================================================================\\
  public DriveSubsystem() {   

  configtalon();
  m_Field2d = new Field2d();
  m_kinematics = new DifferentialDriveKinematics(0.3556);

  m_odometry = new DifferentialDriveOdometry(getRotation2d(), LftEnc(), RgtEnc());

    AutoBuilder.configureRamsete(        
        this::getPose,
        this::resetPose,
        this::getCurrentSpeeds,
        this::driveChassisSpeeds,
        
        new ReplanningConfig(true,false),
        () -> {
        
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
    //RMtrEnc.set(ControlMode.Velocity,-Speed*4096);
    //LMtrEnc.set(ControlMode.Velocity,-Giro*4096);
  }

//==tankdrive==========================================
  public void tanque(double Lft, double Rgt) {
//    RMtrEnc.set(ControlMode.Velocity,-Rgt*4096);
//    LMtrEnc.set(ControlMode.Velocity,-Lft*4096);
    Chasis.tankDrive(-Lft/2.5, -Rgt/2.5);

  /*  RMtrEnc.setVoltage(-Rgt*3.85);
    LMtrEnc.setVoltage(-Lft*3.85);*/
SmartDashboard.putNumber("velL", Lft);
SmartDashboard.putNumber("velR", Rgt);
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
    m_odometry.resetPosition(Navx.getRotation2d(), LftEnc(), RgtEnc(), pose);
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
//==LimeLigt==========================================
  public void Lime_Light() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("LimeLight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    double LL_x = tx.getDouble(0.0);
    double LL_y = ty.getDouble(0.0);
    double LL_a = ta.getDouble(0.0);
      
    SmartDashboard.putNumber("LL x", LL_x);
    SmartDashboard.putNumber("LL y", LL_y);
    SmartDashboard.putNumber("LL a", LL_a);
    SmartDashboard.putNumber("xd", xd);
  }
//==SmartDashboard======================================
  public void Smartdashboard() {
    SmartDashboard.putNumber("Encoder derecho", RgtEnc());
    SmartDashboard.putNumber("Velocidad derecho", RgtVel());

    SmartDashboard.putNumber("Encoder izquuierdo", LftEnc());
    SmartDashboard.putNumber("Velocidad izquierdo", LftVel());


  }

//==Configuracion de los motores y PID==================
  public void configtalon() {    

    //double kF=1023.0/7200.0;
 /*    double LkP=0.15;
    double LkI=0.000;
    double LkD=0;

    double RkP=0.15;
    double RkI=0.000;
    double RkD=0;

    int kPIDLoopIdx = 0;
    int kTimeoutMs=30; */
    
    RMtrFllw.follow(RMtrEnc);
    LMtrFllw.follow(LMtrEnc);
    
    LMtrEnc.setInverted(true);
    
    RMtrEnc.setInverted(false);
    
    RMtrFllw.setInverted(InvertType.FollowMaster);
    LMtrFllw.setInverted(InvertType.FollowMaster);
    
    RMtrEnc.setSensorPhase(true);
    LMtrEnc.setSensorPhase(true);

    RMtrEnc.setNeutralMode(NeutralMode.Brake);
    RMtrFllw.setNeutralMode(NeutralMode.Brake);
    LMtrEnc.setNeutralMode(NeutralMode.Brake);
    LMtrFllw.setNeutralMode(NeutralMode.Brake);

    /* 
    RMtrEnc.configNominalOutputForward(0, kTimeoutMs);
		RMtrEnc.configNominalOutputReverse(0, kTimeoutMs);
		RMtrEnc.configPeakOutputForward(1, kTimeoutMs);
		RMtrEnc.configPeakOutputReverse(-1, kTimeoutMs);

		//RMtrEnc.config_kF(kPIDLoopIdx, kF,kTimeoutMs);
		RMtrEnc.config_kP(kPIDLoopIdx, RkP, kTimeoutMs);
		RMtrEnc.config_kI(kPIDLoopIdx, RkI, kTimeoutMs);
		RMtrEnc.config_kD(kPIDLoopIdx, RkD, kTimeoutMs);

    LMtrEnc.configNominalOutputForward(0, kTimeoutMs);
		LMtrEnc.configNominalOutputReverse(0, kTimeoutMs);
		LMtrEnc.configPeakOutputForward(1, kTimeoutMs);
		LMtrEnc.configPeakOutputReverse(-1, kTimeoutMs);

	  //LMtrEnc.config_kF(kPIDLoopIdx, kF, kTimeoutMs);
		LMtrEnc.config_kP(kPIDLoopIdx, LkP, kTimeoutMs);
		LMtrEnc.config_kI(kPIDLoopIdx, LkI, kTimeoutMs);
		LMtrEnc.config_kD(kPIDLoopIdx, LkD, kTimeoutMs);
    */
  }
}