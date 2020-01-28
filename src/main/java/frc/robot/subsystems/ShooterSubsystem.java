package frc.robot.subsystems;

import javax.swing.plaf.basic.BasicInternalFrameTitlePane.CloseAction;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  private double RPM;
  private TalonSRX shooterMotor = new TalonSRX(Constants.Shooter.kShooterMotorID);
  
  public ShooterSubsystem() {
    // TODO reset talon to defaults
    shooterMotor.setNeutralMode(NeutralMode.Coast);
    shooterMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
  }
 
  public double getSpeed(){
    // TODO check if this is 4096 or something else
    // TODO put the RPM/RPS in a constant
    RPM = sensorUnitsToRPM(shooterMotor.getSelectedSensorVelocity());
    return RPM;
  }

  public void setSpeed(double speed){
    shooterMotor.set(ControlMode.Velocity, speed * 4096);
  }

  public double sensorUnitsToRPM(double sensorVelocity) {
    return (sensorVelocity * 10 * 60)/ 4096; //sensor velocity gives ticks per 100 milliseconds, times by 10 to get seconds, times by 60 to get minutes, divide by encoder ticks per revolution
  }
}
