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

//CIOCCI VENTEREA
//TODO find max and min values/heights for the hood manualy with the robot 
//TODO Lemon Switches???

public class HoodSubsystem extends SubsystemBase {
  private TalonSRX angleMotor = null;
  private boolean runningInOpenLoop = true;

  public HoodSubsystem() {
    angleMotor = new TalonSRX(Constants.Shooter.kAngleMotorID);
    angleMotor.configFactoryDefault();
    angleMotor.setNeutralMode(NeutralMode.Brake);
    angleMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
  }

  public void zeroSensor(){ // to zero hood push it all the way down on the robot
    angleMotor.setSelectedSensorPosition(0, 0, 10);
  }

  //Moves hood up at low incaments with a max postion checker
  public void manualMoveHoodUp(){ 
    //TODO check gear ratios to find better motor speed
    if (angleMotor.getSelectedSensorPosition() >= Constants.Hood.kMaxHoodHight) {
      angleMotor.set(ControlMode.PercentOutput, 0);
    } else {
      angleMotor.set(ControlMode.PercentOutput, Constants.Hood.kHoodUpSpeed);
    }
  }
    
  //Moves Hood Down at low incraments with a min postion checker 
  public void manualMoveHoodDown(){ //TODO check gear ratios to find better motor speed
    if (angleMotor.getSelectedSensorPosition() >= Constants.Hood.kMinHoodHight) {
      angleMotor.set(ControlMode.PercentOutput, 0);
    } else {
      angleMotor.set(ControlMode.PercentOutput, Constants.Hood.kHoodDownSpeed);
    }
  }
  
  //makes the motor stop
  public void manualStopHoodMovement(){
    angleMotor.set(ControlMode.PercentOutput, 0);
  }
  
  //AimHood creates an endoder value that it calculates 
  //from a given angle and ticks per rotation then moves 
  //the motor to that encoder value baised on max and min constriants   
  public void aimHood(int targetAngle ) { 
    //TODO: I think all the motion magic PID values will need to be set, check documentation
    int encoderValue = (targetAngle / 360) * Constants.Hood.kTicksPerRotation;
    if (encoderValue <= Constants.Hood.kMinHoodHight) {
      angleMotor.set(ControlMode.MotionMagic, Constants.Hood.kMinHoodHight);
    } else if (encoderValue >= Constants.Hood.kMaxHoodHight) {
      angleMotor.set(ControlMode.MotionMagic, Constants.Hood.kMaxHoodHight);
    } else angleMotor.set(ControlMode.MotionMagic, encoderValue); 
  }

//getCurrentAngle returns the current angle of the hood
  public double getCurrentAngle() {
    double currentTicks = angleMotor.getSelectedSensorPosition();
    double currentAngle = (currentTicks / Constants.Hood.kTicksPerRotation) * 360;
    return currentAngle;
  }
  
}
