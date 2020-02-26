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
  private boolean wantActiveBalanceRight = false;
  private boolean wantActiveBalanceLeft = false;
  private Timer timer = new Timer();
  
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
     
    timer.start();
  }

  public void setWantActiveBalanceLeft(boolean isPressed) {
    wantActiveBalanceLeft = isPressed;
  }

  public void setWantActiveBalanceRight(boolean isPressed) {
    wantActiveBalanceRight = isPressed;
  }

  public void setWantDown(boolean isPressed) {
    wantConveyorDown = isPressed;
    //System.out.println("--------------------SET WANT DOWN" + isPressed);
  }

  public void setWantUp(boolean isPressed) {
    //System.out.println("--------------------SET WANT UP" + isPressed);
    wantConveyorUp = isPressed;
  }

  public void setWantPreload(boolean isPressed) {
    //System.out.println("----------------------------PRELOAD" + isPressed);
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
      lifterMotor.set(ControlMode.PercentOutput, -.25);
    } else if(wantConveyorDown) {
      lifterMotor.set(ControlMode.PercentOutput, Constants.ConveyorSubsystem.kConveyorSpeed);
    } else {
      lifterMotor.set(ControlMode.PercentOutput, 0);
    }

    if(wantConveyorUp) {
      //conveyorBottomRightMotor.set(ControlMode.PercentOutput, -1);
      //conveyorBottomLeftMotor.set(ControlMode.PercentOutput, 1);
      double time = timer.get();
      if(time %  2 < 1) {
        conveyorBottomRightMotor.set(ControlMode.PercentOutput, -1);
        conveyorBottomLeftMotor.set(ControlMode.PercentOutput, .75);
      // } else if( time % 5 < 2.5) {
      //   conveyorBottomRightMotor.set(ControlMode.PercentOutput, -Constants.ConveyorSubsystem.kConveyorSpeed);
      //   conveyorBottomLeftMotor.set(ControlMode.PercentOutput, Constants.ConveyorSubsystem.kConveyorSpeed);
      // } else if(time % 5 < 4.5) {
      //   conveyorBottomRightMotor.set(ControlMode.PercentOutput, -Constants.ConveyorSubsystem.kConveyorSpeed);
      //   conveyorBottomLeftMotor.set(ControlMode.PercentOutput, -Constants.ConveyorSubsystem.kConveyorSpeed);
      } else {
        conveyorBottomRightMotor.set(ControlMode.PercentOutput, .75);
        conveyorBottomLeftMotor.set(ControlMode.PercentOutput, -1);
      } 
    } else if(wantConveyorDown) {
      conveyorBottomRightMotor.set(ControlMode.PercentOutput, Constants.ConveyorSubsystem.kConveyorSpeed);
      conveyorBottomLeftMotor.set(ControlMode.PercentOutput, Constants.ConveyorSubsystem.kConveyorSpeed);  
    } else if(wantActiveBalanceRight) {
      conveyorBottomRightMotor.set(ControlMode.PercentOutput, .25);
    } else if(wantActiveBalanceLeft) {
      conveyorBottomRightMotor.set(ControlMode.PercentOutput, -.25);
    } else {
      conveyorBottomRightMotor.set(ControlMode.PercentOutput, 0);
      conveyorBottomLeftMotor.set(ControlMode.PercentOutput, 0);
    }
  }



}
