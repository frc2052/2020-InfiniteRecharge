/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FalconShooterSubsystem extends SubsystemBase {

  private WPI_TalonFX shooterMasterMotor;
  private double lastShootPct = 0;
  private boolean isOnTarget = false;
  
  /**
   * Creates a new FalconShooterSubsystem.
   */
  public FalconShooterSubsystem() {
    shooterMasterMotor = new WPI_TalonFX(Constants.Motors.kFalconShooterMotorId);
    shooterMasterMotor.configFactoryDefault();
    shooterMasterMotor.setNeutralMode(NeutralMode.Coast);
    shooterMasterMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);

    shooterMasterMotor.config_kF(0, 0.08, 10);
    shooterMasterMotor.config_kP(0, 0.02, 10);
    shooterMasterMotor.config_kI(0, 0, 10);
    shooterMasterMotor.config_kD(0, 0, 10);
  }

  public void putToSmartDashboard() {
//    System.out.println("Falcon Position " + shooterMasterMotor.getSelectedSensorPosition());
//    System.out.println("Falcon Velocity " + getVelocityTicks());
    SmartDashboard.putNumber("Shooter Velocity", getVelocityTicks());
  }
 
  public double getVelocityTicks(){
    //The Falcon has a "counts per revolution" of 2048.  
    //This is half the standard revotuion of other encoders
    //divide by 2 so the code will work the same if we switch back to 775 Pro shooters
    return shooterMasterMotor.getSelectedSensorVelocity() * 2;
  }

  public boolean getIsOnTarget() {
    return isOnTarget;
  }

  public double getSpeedPct() {
    return lastShootPct;
  }

  public void setShooterVelocity(double speed){
    lastShootPct = 0;    

//    System.out.println("SETTING SHOOTER VELOCITY  " + speed);
    //The Falcon has a "counts per revolution" of 2048.  
    //This is half the standard revotuion of other encoders
    //multiply by 2 so the code will work the same if we switch back to 775 Pro shooters
    
    int falconSpeed = (int)speed/2;
    int falconAdjusted = (int)(falconSpeed * .66);

    shooterMasterMotor.set(ControlMode.Velocity, falconAdjusted);

    isOnTarget = shooterMasterMotor.getSelectedSensorVelocity() > falconAdjusted * .90;

//    shooterMasterMotor.set(ControlMode.Velocity, 6000);
    //System.out.println("SHOOTER VELOCTITY MODE--" +  getVelocityTicks() + "  TARGET VELOCITY--" + speed);
  }

  public double sensorUnitsToRPM(double sensorVelocity) {
    return (sensorVelocity * 10 * 60)/ 4096; //sensor velocity gives ticks per 100 milliseconds, times by 10 to get seconds, times by 60 to get minutes, divide by encoder ticks per revolution
  }

  public void setShooterPct(double pct) {
    isOnTarget = false; //can't be on target when setting by percent
    shooterMasterMotor.set(ControlMode.PercentOutput, pct);
    lastShootPct = pct;
  }

  public void resetEncoder() {
    shooterMasterMotor.setSelectedSensorPosition(0);
  }

}
