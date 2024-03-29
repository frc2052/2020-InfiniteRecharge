package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SevenSevenFiveShooterSubsystem extends SubsystemBase {
  private TalonSRX shooterMasterMotor;
  private VictorSPX shooterFollowerMotor;

  private double lastShootPct;
  
  public SevenSevenFiveShooterSubsystem() {
    shooterMasterMotor = new TalonSRX(Constants.Motors.kShooterMasterMotorID);
    shooterMasterMotor.configFactoryDefault();
    shooterMasterMotor.setNeutralMode(NeutralMode.Coast);
    shooterMasterMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);

    shooterFollowerMotor = new VictorSPX(Constants.Motors.kShooterFollowerMotorID);
    shooterFollowerMotor.configFactoryDefault();

    
    shooterFollowerMotor.setNeutralMode(NeutralMode.Coast);

    shooterFollowerMotor.follow(shooterMasterMotor);

    shooterMasterMotor.setSensorPhase(true);

    double fGain = (.9 * 1023) / 31000;
    shooterMasterMotor.config_kF(0, fGain, 10);
    shooterMasterMotor.config_kP(0, 0, 10);
    shooterMasterMotor.config_kI(0, 0, 10);
    shooterMasterMotor.config_kD(0, 0, 10);
  }

  public void putToSmartDashboard() {
    SmartDashboard.putNumber("Shooter Velocity", shooterMasterMotor.getSelectedSensorVelocity());
  }
 
  public double getVelocityTicks(){
    return shooterMasterMotor.getSelectedSensorVelocity();
  }

  public double getSpeedPct() {
    return lastShootPct;
  }

  public void setShooterVelocity(double speed){
    lastShootPct = 0;
    //System.out.println("SHOOTER VELOCTITY MODE--" +  shooterMasterMotor.getSelectedSensorVelocity() + "  TARGET VELOCITY--" + speed);
    shooterMasterMotor.set(ControlMode.Velocity, speed);
  }

  private double sensorUnitsToRPM(double sensorVelocity) {
    return (sensorVelocity * 10 * 60)/ 4096; //sensor velocity gives ticks per 100 milliseconds, times by 10 to get seconds, times by 60 to get minutes, divide by encoder ticks per revolution
  }

  public void setShooterPct(double pct) {
    shooterMasterMotor.set(ControlMode.PercentOutput, pct);
    lastShootPct = pct;
  }

}
