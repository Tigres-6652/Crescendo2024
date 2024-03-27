package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
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
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class DriveSubsystem extends SubsystemBase {
  //Motores
//Declaracione de los motores derecchos
  TalonFX RgtMtrLdr = new TalonFX(1, "rio");
  TalonFX RgtMtrFllw = new TalonFX(2, "rio");

//Declaracion de los motores izquierdos
  TalonFX LftMtrLdr = new TalonFX(3, "rio");
  TalonFX LftMtrFllw = new TalonFX(4, "rio");

//Declaracion para el control diferencial de los motores que estan en el chasis
  DifferentialDrive Chasis = new DifferentialDrive(LftMtrLdr, RgtMtrLdr);

//NAVX y Odometria 
  AHRS Navx = new AHRS(SPI.Port.kMXP);

  DifferentialDriveOdometry m_odometry;
  DifferentialDriveKinematics m_kinematics;
  Field2d m_Field2d;
  Trajectory traye;
  Rotation2d rot = new Rotation2d();
  
  String lime = "limelight-limee";
  double Limex = LimelightHelpers.getTX(lime);

  private final Joystick FirstD = new Joystick(0);

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

    if(  FirstD.getRawButton(9)){
      LimelightHelpers.setLEDMode_ForceBlink(lime);  
      }else{
          LimelightHelpers.setLEDMode_ForceOff(lime);
        }

    m_Field2d.setRobotPose(getPose());

//SmartDashboard.putString("DAta", LimelightHelpers.)
    SmartDashboard.putString("pose", getPose().toString());
    LimelightHelpers.setPriorityTagID(lime, 3);
  }

//==Metodo para controla el chasis=====================
  public void Arcade_Drive(double Speed, double Giro){
    Chasis.arcadeDrive(Speed, Giro);
/*  var forward = -Speed;
    var turn = Giro;
    var leftout = forward + turn;
    var rightout = forward - turn;*/
    //m_leftLeader.setControl(m_leftRequest.withOutput(leftout));
    //m_rightLeader.setControl(m_rightRequest.withOutput(rightout));
    
    //RMtrEnc.set(ControlMode.Velocity,-Speed*4096);
    //LMtrEnc.set(ControlMode.Velocity,-Giro*4096);
  }

//==tankdrive==========================================
  public void tanque(double Lft, double Rgt) {
//    RMtrEnc.set(ControlMode.Velocity,-Rgt*4096);
//    LMtrEnc.set(ControlMode.Velocity,-Lft*4096);
   // Chasis.tankDrive(-Lft/2.5, -Rgt/2.5);

    RgtMtrLdr.setVoltage(-Rgt*3.85);
    LftMtrLdr.setVoltage(-Lft*3.85);
  //m_leftLeader.setControl(m_leftRequest.withOutput(Lft));
  //m_rightLeader.setControl(m_rightRequest.withOutput(Rgt));

SmartDashboard.putNumber("velL", Lft);
SmartDashboard.putNumber("velR", Rgt);
  }

//==Ecoders============================================
  public double RgtEnc() {
    return (RgtMtrLdr.getPosition().getValue() / 4096 * Math.PI * 6 * 2.54) / 100;
  }
  public double LftEnc() {
    return (LftMtrLdr.getPosition().getValue() / 4096 * Math.PI * 6 * 2.54) / 100;
  }

//==Velocidad del los motores===========================
  public double RgtVel() {
    return (RgtMtrLdr.getVelocity().getValue() / 4096 * Math.PI * 6 * 2.54) / 10;
  }
  public double LftVel() {
    return (LftMtrLdr.getVelocity().getValue() / 4096 * Math.PI * 6 * 2.54) / 10;
  }

//Lectura de rotacion Navx
  public Rotation2d getRotation2d() {
    return Navx.getRotation2d();
  }

//==ResetEncoders=======================================
  public void Reset() {
    //RMtrEnc.setSelectedSensorPosition(0);
    //LMtrEnc.setSelectedSensorPosition(0);
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
    NetworkTable table = NetworkTableInstance.getDefault().getTable("");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    double LL_x = tx.getDouble(0.0);
    double LL_y = ty.getDouble(0.0);
    double LL_a = ta.getDouble(0.0);
      
    SmartDashboard.putNumber("LL x", LL_x);
    SmartDashboard.putNumber("LL y", LL_y);
    SmartDashboard.putNumber("LL a", LL_a);
    SmartDashboard.putNumber("Limex", Limex);
  }

//==SmartDashboard======================================
  public void Smartdashboard() {
    SmartDashboard.putNumber("Encoder derecho", RgtEnc());
    SmartDashboard.putNumber("Velocidad derecho", RgtVel());

    SmartDashboard.putNumber("Encoder izquuierdo", LftEnc());
    SmartDashboard.putNumber("Velocidad izquierdo", LftVel());

    SmartDashboard.putNumber("I der", RgtMtrLdr.getSupplyCurrent().getValue());
    SmartDashboard.putNumber("I Izq", LftMtrLdr.getSupplyCurrent().getValue());
    }

//==Configuracion de los motores y PID==================
  public void configtalon() {    
    var LeftConfiguration = new TalonFXConfiguration();
    var RigtConfiguration = new TalonFXConfiguration();

    LeftConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    RigtConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    LftMtrLdr.getConfigurator().apply(LeftConfiguration);
    LftMtrFllw.getConfigurator().apply(LeftConfiguration);
    RgtMtrLdr.getConfigurator().apply(RigtConfiguration);
    RgtMtrFllw.getConfigurator().apply(RigtConfiguration);

    LftMtrFllw.setControl(new Follower(LftMtrLdr.getDeviceID(), false));
    RgtMtrFllw.setControl(new Follower(RgtMtrLdr.getDeviceID(), false));
  
    LftMtrLdr.setSafetyEnabled(true);
    RgtMtrLdr.setSafetyEnabled(true);
  }
}