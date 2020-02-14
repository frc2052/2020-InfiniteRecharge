/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ConveyorSubsystem extends SubsystemBase {
  private VictorSPX conveyorBottomLeftMotor;
  private VictorSPX conveyorBottomRightMotor;
  private VictorSPX lifterMotor;

  private boolean wantPreload = false;
  private boolean wantConveyorUp = false;
  private boolean wantConveyorDown = false;
  
  public ConveyorSubsystem() {   
    conveyorBottomLeftMotor = new VictorSPX(Constants.Motors.kConveyorMotorBottemLeftID);
    conveyorBottomRightMotor = new VictorSPX(Constants.Motors.kConveyorMotorBottemRightID);
    lifterMotor = new VictorSPX(Constants.Motors.kLifterMotorID);
    lifterMotor.configFactoryDefault();
    conveyorBottomLeftMotor.configFactoryDefault();
    conveyorBottomRightMotor.configFactoryDefault();
  
    lifterMotor.setInverted(!Constants.ConveyorSubsystem.klifterdirection);
    conveyorBottomLeftMotor.setInverted(Constants.ConveyorSubsystem.kConveyorDirection);
    conveyorBottomRightMotor.setInverted(!Constants.ConveyorSubsystem.kConveyorDirection);

    lifterMotor.setNeutralMode(NeutralMode.Coast);
    conveyorBottomLeftMotor.setNeutralMode(NeutralMode.Coast);
    conveyorBottomRightMotor.setNeutralMode(NeutralMode.Coast);
     
  }
  
  // public void lifterUp (){
  //   conveyorBottomLeftMotor.set(ControlMode.PercentOutput, .5);
  //   conveyorBottomRightMotor.set(ControlMode.PercentOutput, -.5);
  //   lifterMotor.set(ControlMode.PercentOutput, -.4);
  // }

  // public void lifterDown (){
  //   conveyorBottomRightMotor.set(ControlMode.PercentOutput, -Constants.ConveyorSubsystem.kConveyorSpeed);
  //   conveyorBottomLeftMotor.set(ControlMode.PercentOutput, -Constants.ConveyorSubsystem.kConveyorSpeed);
  //   lifterMotor.set(ControlMode.PercentOutput, -Constants.ConveyorSubsystem.kConveyorSpeed);
  // }

  // private void setLeftSideSpeed(double speed) {
  //   conveyorBottomLeftMotor.set(ControlMode.PercentOutput, speed);
  // }

  // private void setRightSideSpeed(double speed) {
  //   conveyorBottomRightMotor.set(ControlMode.PercentOutput, speed);
  // }

  // public void lifterStop (){
  //   conveyorBottomRightMotor.set(ControlMode.PercentOutput, 0);
  //   conveyorBottomLeftMotor.set(ControlMode.PercentOutput, 0);
  //   lifterMotor.set(ControlMode.PercentOutput, 0);
  // }

  public void setWantDown(boolean isPressed) {
    wantConveyorDown = isPressed;
  }

  public void setWantUp(boolean isPressed) {
    wantConveyorUp = isPressed;
  }

  public void setWantPreload(boolean isPressed) {
    wantPreload = isPressed;
  }

  // public void preLoad() {
  //   lifterMotor.set(ControlMode.PercentOutput, -.2);
  // }

  @Override
  public void periodic() {
    if(wantConveyorUp) {
      lifterMotor.set(ControlMode.PercentOutput, -1);
    } else if(wantPreload) {
      lifterMotor.set(ControlMode.PercentOutput, -.2);
    } else if(wantConveyorDown) {
      lifterMotor.set(ControlMode.PercentOutput, -Constants.ConveyorSubsystem.kConveyorSpeed);
    } else {
      lifterMotor.set(ControlMode.PercentOutput, 0);
    }

    Timer timer = new Timer();

    if(wantConveyorUp) {
      double time = timer.get();
      if(time %  5 < 2) {
        conveyorBottomRightMotor.set(ControlMode.PercentOutput, -Constants.ConveyorSubsystem.kConveyorSpeed);
        conveyorBottomLeftMotor.set(ControlMode.PercentOutput, -Constants.ConveyorSubsystem.kConveyorSpeed);
      } else if( time % 5 < 2.5) {
        conveyorBottomRightMotor.set(ControlMode.PercentOutput, -Constants.ConveyorSubsystem.kConveyorSpeed);
        conveyorBottomLeftMotor.set(ControlMode.PercentOutput, Constants.ConveyorSubsystem.kConveyorSpeed);
      } else if(time % 5 < 4.5) {
        conveyorBottomRightMotor.set(ControlMode.PercentOutput, -Constants.ConveyorSubsystem.kConveyorSpeed);
        conveyorBottomLeftMotor.set(ControlMode.PercentOutput, -Constants.ConveyorSubsystem.kConveyorSpeed);
      } else {
        conveyorBottomRightMotor.set(ControlMode.PercentOutput, Constants.ConveyorSubsystem.kConveyorSpeed);
        conveyorBottomLeftMotor.set(ControlMode.PercentOutput, -Constants.ConveyorSubsystem.kConveyorSpeed);
      } 
    } else if(wantConveyorDown) {
      conveyorBottomRightMotor.set(ControlMode.PercentOutput, -Constants.ConveyorSubsystem.kConveyorSpeed);
      conveyorBottomLeftMotor.set(ControlMode.PercentOutput, -Constants.ConveyorSubsystem.kConveyorSpeed);
      lifterMotor.set(ControlMode.PercentOutput, -Constants.ConveyorSubsystem.kConveyorSpeed);
    } else {
      conveyorBottomRightMotor.set(ControlMode.PercentOutput, 0);
      conveyorBottomLeftMotor.set(ControlMode.PercentOutput, 0);
      lifterMotor.set(ControlMode.PercentOutput, 0);
    }
  }



}
