/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.Constants;

public class HoodSubsystem extends SubsystemBase {
  private TalonSRX angleMotor;

  public HoodSubsystem() {
    angleMotor = new TalonSRX(Constants.Motors.kHoodMotorID);
    angleMotor.configFactoryDefault();
    angleMotor.setNeutralMode(NeutralMode.Brake);
    angleMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    angleMotor.configClearPositionOnLimitR(true, 10);

    angleMotor.config_kF(0, 0.00001, 10);
    angleMotor.config_kP(0, 0, 10);
    angleMotor.config_kI(0, 0, 10);
    angleMotor.config_kD(0, 0, 10);
  }

  public void resetEncoder(){ 
    angleMotor.setSelectedSensorPosition(0, 0, 10);
  }

  public void manualMoveHoodUp(){ 
    if (angleMotor.getSelectedSensorPosition() >= Constants.Hood.kMaxHoodTicks) {
      angleMotor.set(ControlMode.PercentOutput, 0);
     } else {
      angleMotor.set(ControlMode.PercentOutput, Constants.Hood.kHoodUpSpeed);
    }
  }

  public void manualMoveHoodDown(){ 
    if (angleMotor.getSelectedSensorPosition() <= Constants.Hood.kMinHoodTicks) {
      angleMotor.set(ControlMode.PercentOutput, 0);
    } else {
      angleMotor.set(ControlMode.PercentOutput, Constants.Hood.kHoodDownSpeed);
    }
  }
  
  public void manualStopHoodMovement(){
    angleMotor.set(ControlMode.PercentOutput, 0);
  }

  public double getCurrentTicks() {
    return angleMotor.getSelectedSensorPosition();
  }
  
}
