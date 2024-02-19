package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
//declaracion de los motores Neo
  CANSparkMax RgtMtrArm = new CANSparkMax(7, MotorType.kBrushless);
  CANSparkMax LftMtrArm = new CANSparkMax(8, MotorType.kBrushless);

//Declaracion de los motores
  DigitalInput FrwrdLmt = new DigitalInput(8);
  DigitalInput BckLmt = new DigitalInput(9);

//Control del PID
  ProfiledPIDController PIDRgtMtrArm = new ProfiledPIDController(0.05, 0.0005, 0.00, new TrapezoidProfile.Constraints(80,90));
  ProfiledPIDController PIDLftMtrArm = new ProfiledPIDController(0.05, 0.0005, 0.00, new TrapezoidProfile.Constraints(80,90));

//Numerode Vueltas Motor derecho
  public void RgtPstnVrbl (double Vueltas1) {
    var signal= RgtMtrArm.getEncoder();
    double RgtNeoEnc = signal.getPosition();
    RgtMtrArm.set(PIDRgtMtrArm.calculate(RgtNeoEnc, Vueltas1));
    SmartDashboard.putNumber("Posicion Abajo", RgtNeoEnc);
  }

//Numero de vueltas Motor Izquierdo
  public void LftPstnVrbl (double Vueltas2) {
    var signal= LftMtrArm.getEncoder();
    double LftNeoEnc = signal.getPosition();
    LftMtrArm.set(PIDLftMtrArm.calculate(LftNeoEnc, Vueltas2));
    SmartDashboard.putNumber("Posicion Abajo", LftNeoEnc);
  }

//Metodo para invertir y seguimiente de los motores
  public void MtrArmInvrtAndFllw (){
    RgtMtrArm.setInverted(false);
    LftMtrArm.setInverted(true);

    RgtMtrArm.follow(LftMtrArm);
    LftMtrArm.follow(RgtMtrArm);
  }

//Convercion de vueltas a grados
  public double Axis_Degrees (){
    double PulSnsr = RgtMtrArm.getEncoder().getPosition();
    double ShftTrnRdctn = ((PulSnsr) / (100));
    double AxisDgrs = (ShftTrnRdctn * 0.006975 * (360)) ;
    return AxisDgrs;

  }

//grados en pulsos
  public double ArmAxisPulses (double AxisPulses) {
    double pulses = ((((((AxisPulses) / 360) * 0.006975) * 100)));
    return pulses;
  }

//Bloqueo de Los motores
  public void stop (double speed){
    RgtMtrArm.set(0);

  }

//Limite y moviento libre de los motores
  public void MtrMotionAndLimit (Double SptArm){
    double x = 0;
    double y = 0;

    double a = 0;
    double b = 0;
    double gradosmaximos = 0;
    double limiteposterior = 0;

      if (BckLmt.get() && Axis_Degrees() > -y) {
        RgtMtrArm.set(-SptArm);
        LftMtrArm.set(-SptArm);

      } else if (!BckLmt.get() && SptArm < 0.001) {
        RgtMtrArm.set(-SptArm);
        LftMtrArm.set(-SptArm);
  
      } else if (!BckLmt.get() && SptArm > 0.001) {
        RgtMtrArm.set(0);
        LftMtrArm.set(0);
  
      } else if (Axis_Degrees() < -x && SptArm > 0.001 && BckLmt.get()) {
        RgtMtrArm.set(-SptArm);
        LftMtrArm.set(-SptArm);

      } else if (Axis_Degrees() < -x && SptArm < 0.001 && BckLmt.get()) {
        RgtMtrArm.set(0);
        LftMtrArm.set(0);
  
      }  

      if (FrwrdLmt.get() && Axis_Degrees() > b) {
        RgtMtrArm.set(-SptArm);
        LftMtrArm.set(-SptArm);
  
      } else if (!FrwrdLmt.get() && SptArm < gradosmaximos) {
        RgtMtrArm.set(-SptArm);
        LftMtrArm.set(-SptArm);

      } else if (!FrwrdLmt.get() && SptArm > gradosmaximos) {
        RgtMtrArm.set(0);
        LftMtrArm.set(0);
  
      } else if (Axis_Degrees() < a && SptArm > gradosmaximos && FrwrdLmt.get()) {
        RgtMtrArm.set(-SptArm);
        LftMtrArm.set(-SptArm);

      } else if (Axis_Degrees() < a && SptArm < gradosmaximos && FrwrdLmt.get()) {
        RgtMtrArm.set(0);
        LftMtrArm.set(0);
  
      }  

      if (!FrwrdLmt.get()) {
        RgtMtrArm.getEncoder().setPosition(limiteposterior);
        LftMtrArm.getEncoder().setPosition(limiteposterior);
      }

      if (!BckLmt.get()) {
        RgtMtrArm.getEncoder().setPosition(0);
        LftMtrArm.getEncoder().setPosition(0);
      }
      
    }

//Regreso A casita
  public void RtrnHm (boolean status) {
    if (status) {

     if (!BckLmt.get()) {
        RgtMtrArm.set(0);
       LftMtrArm.set(0);

     } else {
       RgtMtrArm.set(-0.8);
       LftMtrArm.set(-0.8);

       }
     }
    }

//Controles para pruebas
  public void Botones (boolean Boton_0, boolean Boton_90){
    if((Boton_0)){
      RgtPstnVrbl(0);

    }else if (Boton_90){
      RgtPstnVrbl(90);

    }else{
      stop(0);
    }

  }

  public ArmSubsystem() {}

  @Override
  public void periodic() {}

/*
  TalonFX MAmr1 = new TalonFX (7);
  
  ProfiledPIDController pidController =new ProfiledPIDController
  (0.17, 0.17, 0.0025, new TrapezoidProfile.Constraints(80, 90));

  public void PosicionVariable (double Grado1) {
    var signal= MAmr1.getPosition();
    double FxEnc1 = signal.getValue();
    MAmr1.set(pidController.calculate(FxEnc1, Grado1));
    SmartDashboard.putNumber("Posicion Abajo", FxEnc1);
  } 
*/

/*Calculos para la reduccion
*
 * Reducciones 
 * 50
 * 48
 * 
 * 1.041666
 * 
 * 64
 * 15
 * 0.234675
 * 
 * 1:5
 * 1:7
 * 
 * 1:35
 * 0.028571
 * 
 * total
 * 0.006975
 * 
 */
 /*⢠⡶⠛⠛⠢⣄⠀⠀⣀⣀⣀⣤⣀⣀⣀⢀⡤⠖⠛⠓⣆⠀⠀⠀
⠀⠀⣾⠁⢠⡆⠀⣌⠿⠿⠛⣻⣿⣯⠙⣛⠻⠏⠀⣠⣤⡀⢸⠀⠀⠀
⠀⠀⢹⣤⣻⠏⠚⢉⣠⣾⡵⠭⢻⠂⠠⣭⣓⠦⣀⠙⢻⣷⠟⠀⠀⠀
⠀⠀⠈⢯⠤⡤⢞⡧⠈⡵⠃⡞⢻⡈⠳⡙⢦⡘⢧⡓⢄⢾⠀⠀⠀⠀
⠀⠀⢰⠃⡾⢡⡏⡧⢾⢄⠸⠡⠛⠋⠒⠛⡠⣽⢆⢳⡘⡎⢢⠀⠀⠀
⠀⢠⡇⢸⡇⣿⢰⢻⣶⣾⡷⡄⠀⠀⠀⣾⣟⣿⡹⠋⡇⣧⠀⢇⠀⠀
⠀⢸⠀⢸⡇⢻⡸⢦⣙⡊⣿⠁⠀⠀⠀⢻⡉⠉⠀⠀⡇⣿⠀⢸⣷⣄
⠀⡾⠀⢸⣧⠘⡟⠂⠲⣤⡿⠀⠀⠀⠀⠈⠓⣜⣶⢶⠁⣿⠁⢸⣿⡏
⢀⣿⡀⠈⢻⣄⢸⡌⢷⡎⠀⠀⠀⠀⠀⠀⠆⠈⣯⣄⣤⡿⠀⢸⠁⢀
⠈⠘⣷⡀⠀⢿⣷⣿⣟⣻⡤⡷⣶⡒⡶⠞⣠⣾⣿⡶⠿⠋⢐⠆⠀⣼
⠀⠀⠹⣿⡇⠀⡎⡏⡿⣯⢽⡟⠈⣻⡁⠠⢿⣿⠟⠁⢀⢀⡾⠀⢰⡟
⠀⠀⠀⣿⣧⣀⢀⣿⢠⠈⡻⠓⠉⠉⠉⠒⢾⠙⡄⢱⣤⡿⠄⣠⡿⠃
⠀⠀⢀⡸⡛⠿⣧⣉⡋⠛⠧⢄⣀⣀⡀⣠⠾⢯⠴⠿⠏⣠⣾⣿⠃⠀
⠀⠀⠈⢳⡕⣄⠀⠙⠻⢶⣤⡀⠉⠙⠻⡁⠀⠀⠀⢀⡼⣻⣿⠃⠀⠀
⠀⠀⠀⠀⠙⣮⠑⠦⣀⠀⠉⠻⢶⣄⠀⠈⠦⡀⠖⠉⢰⣿⠋⠀⠀⠀
⠀⠀⠀⠀⠀⠈⠳⣤⣀⠉⠓⠄⠀⠙⢿⣤⡀⠀⠀⣠⡿⠁⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠈⠛⠷⣶⣤⡀⠀⠈⠻⡛⠂⠀⣉⠀⠀⠀⠀⠀⠀ 
*/
}
