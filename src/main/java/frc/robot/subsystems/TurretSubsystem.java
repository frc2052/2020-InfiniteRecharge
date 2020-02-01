/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.lang.invoke.ConstantBootstraps;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {
  private TalonSRX turretMotor = new TalonSRX(Constants.Turret.kTurretMotorID);
  private int currentPos = 0;
  private double motorPower = 0;
 
  public TurretSubsystem() {
    turretMotor.configFactoryDefault();
    turretMotor.setNeutralMode(NeutralMode.Brake);
    turretMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);

    // turretMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 10);
    // turretMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 10);

    // /* set the peak and nominal outputs */
    // turretMotor.configNominalOutputForward(0, 10);
    // turretMotor.configNominalOutputReverse(0, 10);
    // turretMotor.configPeakOutputForward(.5, 10);
    // turretMotor.configPeakOutputReverse(-.5, 10);

    // /* set closed loop gains in slot0 - see documentation */
    // turretMotor.selectProfileSlot(0, 0);
    // turretMotor.config_kF(0, 0.2, 10); //1843
    // turretMotor.config_kP(0, .8, 10);
    // turretMotor.config_kI(0, 0, 10);
    // turretMotor.config_kD(0, 0, 10);

    // /* set acceleration and vcruise velocity - see documentation */
    // turretMotor.configMotionCruiseVelocity((int)(5550 * .9), 10);
    // turretMotor.configMotionAcceleration((int)(5550 * .9 * 2), 10);
    /* zero the sensor */
    turretMotor.setSelectedSensorPosition(0, 0, 10);
  }

  

  public void turnTurret(double power){
    //motorPower = power;
    turretMotor.set(ControlMode.PercentOutput, power);
  }

  @Override
  public void periodic() {
    // // This method will be called once per scheduler run
    // // TODO do we need to convert this to degrees?
    // currentPos = turretMotor.getSelectedSensorPosition();
    // if (currentPos > Constants.Turret.kTurretMaxRight && motorPower > 0){
    //   motorPower = 0;
    // } else if (currentPos < Constants.Turret.kTurretMinLeft && motorPower < 0){
    //   motorPower = 0;
    // }
    // turretMotor.set(ControlMode.PercentOutput, motorPower);
    
  }
  
  // takes angle and drives to encoder pos
  public void driveToPos(double angle) {

    System.out.println("****************** TARGET ANGLE: " + angle);

    if(Math.abs(angle) < 0.2) {
      turretMotor.set(ControlMode.PercentOutput, 0);
   } else {
    if (Math.abs(angle) >= 20) {
      if(angle > 0) {
        turretMotor.set(ControlMode.PercentOutput, 1);
      } else {
        turretMotor.set(ControlMode.PercentOutput, -1);
      }
    } else if (Math.abs(angle) < 20) {
        turretMotor.set(ControlMode.PercentOutput, (angle * 0.05));
    }
   }

    

    // if(angle <= Constants.Turret.kMaxAngle && angle >= Constants.Turret.kMinAngle) {
    //   ticks = angle * Constants.Turret.kTicksPerDegree;
    // } else {
    //   if(angle > Constants.Turret.kMaxAngle)
    //     ticks = Constants.Turret.kMaxAngle * Constants.Turret.kTicksPerDegree;
    //   else
    //     ticks = Constants.Turret.kMinAngle * Constants.Turret.kTicksPerDegree;
    // }
    // System.out.println("-----------------------------------Current: " + turretMotor.getSelectedSensorPosition() + "    Target: " + ticks);
    // turretMotor.set(ControlMode.MotionMagic, ticks);
  }

  public double getTurretDegree() {
    double ticks = turretMotor.getSelectedSensorPosition(0);
    return ticks / Constants.Turret.kTicksPerDegree;
  }

  public void printEncoderPos() {
    System.out.println(turretMotor.getSelectedSensorPosition());
  }

}

