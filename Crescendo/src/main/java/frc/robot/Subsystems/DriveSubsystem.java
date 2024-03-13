package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
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
  private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(20.5));

//=================================================================================================================\\

//Configuracion de los motores y navx
//Seguimiento e inversion de los motores e inicialisacion de la odometria
  public DriveSubsystem() {
    RMtrEnc.setInverted(false);
    LMtrEnc.setInverted(true);

    RMtrFllw.follow(RMtrEnc);
    LMtrFllw.follow(LMtrEnc);
  
    m_odometry = new DifferentialDriveOdometry(getRotation2d(), LftEnc(), RgtEnc());


    AutoBuilder.configureRamsete(
      this::getPose, // Robot pose supplier
      this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getWheelSpeeds, // Current ChassisSpeeds supplier
      this::driveVelocity, // Method that will drive the robot given ChassisSpeeds
      new ReplanningConfig(), // Default path replanning config. See the API for the options here
      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      this // Reference to this subsystem to set requirements
);
}

public void configtalons(){

  RMtrEnc.configFactoryDefault();
  RMtrFllw.configFactoryDefault();
  LMtrEnc.configFactoryDefault();
  LMtrFllw.configFactoryDefault();

  RMtrFllw.follow(RMtrEnc);
  LMtrFllw.follow(LMtrEnc);

  LMtrEnc.setInverted(true);
  RMtrEnc.setInverted(false);

  RMtrFllw.setInverted(InvertType.FollowMaster);
  LMtrFllw.setInverted(InvertType.FollowMaster);

  RMtrEnc.setSensorPhase(true);
  LMtrEnc.setSensorPhase(true);



  }

//Metodo para controla el chasis
  public void Arcade_Drive(double Speed, double Giro){
    Chasis.arcadeDrive(Speed, Giro);

  }

  DifferentialDriveKinematics cinematics = new DifferentialDriveKinematics(27.0 * 2.54 / 100);    

//=================================================================================================================\\


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

//SmartDashboard de los enocders
  public void SDEncders () {
    SmartDashboard.putNumber("Distancia encoder derecho", RgtEnc());
    SmartDashboard.putNumber("Distancia encoder izquierdo", LftEnc());
  

    
  }

  
//=================================================================================================================\\

//Configuraciones Navx
//Plano 2D
  public Rotation2d getRotation2d() {
    return Navx.getRotation2d();
  }


//Posicion en metros
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

//Reseteo de la posicion
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(getRotation2d(), LftEnc(), RgtEnc(), pose);
  }

  public ChassisSpeeds getWheelSpeeds(){
    DifferentialDriveWheelSpeeds wheelSpeeds =  new DifferentialDriveWheelSpeeds(LftEnc(),RgtEnc());
    return kinematics.toChassisSpeeds(wheelSpeeds);
}

    public void driveVelocity(ChassisSpeeds speed){
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speed);
        SmartDashboard.putNumber("LeftVelocitySetpoint", wheelSpeeds.leftMetersPerSecond);
        SmartDashboard.putNumber("RightVelocitySetpoint", wheelSpeeds.rightMetersPerSecond);

        
    }
//Resetiado de la navx
  public void zeroHeading() {
    Navx.reset();
  }

//=================================================================================================================\\

//Reseteo de los sensores
  public void Reset() {
    zeroHeading();
    resetEncoders();
  }

//=================================================================================================================\\

  @Override
  public void periodic() {

SDEncders();

  }




}
