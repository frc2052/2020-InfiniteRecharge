package frc.robot.subsystems;

import javax.swing.plaf.basic.BasicInternalFrameTitlePane.CloseAction;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  /**
   * Creates a new ShooterSubsystem.
   */
  private double RPM;
  private TalonSRX shooterMotor = new TalonSRX(Constants.Shooter.kShooterMotorID);
  
  public ShooterSubsystem() {
    shooterMotor.setNeutralMode(NeutralMode.Coast);
    shooterMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
  }
 
  public double getSpeed(){
    RPM = shooterMotor.getSelectedSensorVelocity()/4096;
    return RPM;
  }
  public void setSpeed(double speed){
    shooterMotor.set(ControlMode.Velocity, speed * 4096);
  }

  @Override
  public void periodic() {
  }
}
