package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
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
import edu.wpi.first.wpilibj.Encoder;
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
  TalonFX RgtMtrLdr = new TalonFX(3, "rio");
  TalonFX RgtMtrFllw = new TalonFX(4, "rio");

//Declaracion de los motores izquierdos
  TalonFX LftMtrLdr = new TalonFX(1, "rio");
  TalonFX LftMtrFllw = new TalonFX(2, "rio");

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
  double giro;
  private final Joystick FirstD = new Joystick(0);

  


  //List<PathPlannerAuto> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile("si");
  //private Pose2d posee = PathPlannerAuto.getStaringPoseFromAutoFile("si");

      double kplime=0.4;
      double Limex = LimelightHelpers.getTX(lime)*kplime;


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
        this::getalliance,
      this);
  }

  @Override
  public void periodic() {
    m_odometry.update(getRotation2d(), LftEnc(), RgtEnc());
    Smartdashboard();

    SmartDashboard.putBoolean("alliance", getalliance());

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
    Chasis.arcadeDrive( Speed,-Giro);



/*  var forward = -Speed;
    var turn = Giro;
    var leftout = forward + turn;
    var rightout = forward - turn;*/
    //m_leftLeader.setControl(m_leftRequest.withOutput(leftout));
    //m_rightLeader.setControl(m_rightRequest.withOutput(rightout));
    
    //RMtrEnc.set(ControlMode.Velocity,-Speed*4096);
    //LMtrEnc.set(ControlMode.Velocity,-Giro*4096);

  }

  public void AimAndDist(double vel){

    double velLim=0.4;
 

    if(Limex>velLim){

      giro=velLim;

    }else if(Limex<velLim && Limex > -velLim){

      giro=Limex;

    }else{

      giro=-velLim;

    }

    Chasis.arcadeDrive(vel, giro);

  }

  //False is Blue, True is Red
  public boolean getalliance(){
    
        var alliance = DriverStation.getAlliance();
         if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false; 
  }

  public void configprioritylime(){

    if(getalliance()){
      //Red
      LimelightHelpers.setPriorityTagID(lime, 4);

    }else{
      //Blue
      LimelightHelpers.setPriorityTagID(lime, 8);

    }

  }

//==tankdrive==========================================
  public void tanque(double Lft, double Rgt) {
//    RMtrEnc.set(ControlMode.Velocity,-Rgt*4096);
//    LMtrEnc.set(ControlMode.Velocity,-Lft*4096);
   // Chasis.tankDrive(-Lft/2.5, -Rgt/2.5);

    RgtMtrLdr.setVoltage(-Rgt*3.85);
    LftMtrLdr.setVoltage(-Lft*3.85);

    SmartDashboard.putNumber("velL", Lft);
    SmartDashboard.putNumber("velR", Rgt);
  }

//==Ecoders============================================
  public double RgtEnc() {
    return (((RgtMtrLdr.getPosition().getValue()) + (RgtMtrFllw.getPosition().getValue())  / 2) *0.88  /6.05) ;
  }
  public double LftEnc() {
    return (((LftMtrLdr.getPosition().getValue()) + (LftMtrFllw.getPosition().getValue())  / 2)  * Math.PI * 6 * 2.54) / 100;
  }

//==Velocidad del los motores===========================
  public double RgtVel() {
    return (((RgtMtrLdr.getVelocity().getValue()) + (RgtMtrFllw.getVelocity().getValue()) / 2)  * Math.PI * 6 * 2.54) / 10;
  }
  public double LftVel() {
    return (((LftMtrLdr.getVelocity().getValue()) + (LftMtrFllw.getVelocity().getValue()) / 2)  * Math.PI * 6 * 2.54) / 10;
  }

//Lectura de rotacion Navx
  public Rotation2d getRotation2d() {
    return Navx.getRotation2d();
  }

//==ResetEncoders=======================================
  public void Reset() {
    RgtMtrLdr.setPosition(0);
    RgtMtrFllw.setPosition(0);
    LftMtrLdr.setPosition(0);
    LftMtrFllw.setPosition(0);
    
    Navx.reset();

    //RMtrEnc.setSelectedSensorPosition(0);
    //LMtrEnc.setSelectedSensorPosition(0);
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
  //seguidor del motor
    var LeftConfiguration = new TalonFXConfiguration();
    var RigtConfiguration = new TalonFXConfiguration();

    LeftConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    RigtConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    

    LftMtrLdr.getConfigurator().apply(LeftConfiguration);
    LftMtrFllw.getConfigurator().apply(LeftConfiguration);
    RgtMtrLdr.getConfigurator().apply(RigtConfiguration);
    RgtMtrFllw.getConfigurator().apply(RigtConfiguration);

    LftMtrFllw.setControl(new Follower(LftMtrLdr.getDeviceID(), false));
    RgtMtrFllw.setControl(new Follower(RgtMtrLdr.getDeviceID(), false));
  
    LftMtrLdr.setSafetyEnabled(true);
    RgtMtrLdr.setSafetyEnabled(true);


  //limites de corriente
    //CurrentLimitsConfigs m_currentLimits = new CurrentLimitsConfigs();
   // TalonFXConfiguration toConfigure = new TalonFXConfiguration();
/* 
    m_currentLimits.SupplyCurrentLimit = 1; // Limit to 1 amps
    m_currentLimits.SupplyCurrentThreshold = 4; // If we exceed 4 amps
    m_currentLimits.SupplyTimeThreshold = 1.0; // For at least 1 second
    m_currentLimits.SupplyCurrentLimitEnable = true; // And enable it

    m_currentLimits.StatorCurrentLimit = 20; // Limit stator to 20 amps
    m_currentLimits.StatorCurrentLimitEnable = true; // And enable it

    toConfigure.CurrentLimits = m_currentLimits;
*/
 //   LftMtrLdr.getConfigurator().apply(toConfigure);
   // LftMtrFllw.getConfigurator().apply(toConfigure);
    //RgtMtrLdr.getConfigurator().apply(toConfigure);
    //RgtMtrFllw.getConfigurator().apply(toConfigure);
  }

  

}