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
  private boolean wantManualConveyorUp = false;
  private boolean wantConveyorDown = false;
  private boolean wantActiveBalanceRight = false;
  private boolean wantActiveBalanceLeft = false;
  private Timer timer = new Timer();
  
  public ConveyorSubsystem() {   
    conveyorBottomLeftMotor = new VictorSPX(Constants.Motors.kConveyorMotorBottemLeftID);
    conveyorBottomRightMotor = new VictorSPX(Constants.Motors.kConveyorMotorBottemRightID);
    lifterMotor = new VictorSPX(Constants.Motors.kLifterMotorID);
    
    conveyorBottomLeftMotor.configFactoryDefault();
    conveyorBottomRightMotor.configFactoryDefault();
    lifterMotor.configFactoryDefault();
  
    conveyorBottomLeftMotor.setInverted(Constants.ConveyorSubsystem.kConveyorDirection);
    conveyorBottomRightMotor.setInverted(!Constants.ConveyorSubsystem.kConveyorDirection);
    lifterMotor.setInverted(!Constants.ConveyorSubsystem.klifterdirection);

    conveyorBottomLeftMotor.setNeutralMode(NeutralMode.Brake);
    conveyorBottomRightMotor.setNeutralMode(NeutralMode.Brake);
    lifterMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void setWantDown(boolean isPressed) {
    wantConveyorDown = isPressed;
  }

  public void setWantUp(boolean isPressed) {
    wantConveyorUp = isPressed;
  }

  public void setWantManualUp(boolean isPressed) {
    wantManualConveyorUp = isPressed;
  }

  public void setWantPreload(boolean isPressed) {
    wantPreload = isPressed;
  }

  @Override
  public void periodic() {

    if(wantConveyorUp) {
      System.out.println("CONVEYOR UP");
      lifterMotor.set(ControlMode.PercentOutput, -1);
    } else if(wantManualConveyorUp) {
      System.out.println("CONVEYOR MANUAL UP");
      lifterMotor.set(ControlMode.PercentOutput, -.5);
    }else if(wantPreload) {
      System.out.println("CONVEYOR PRELOAD");
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
      conveyorBottomRightMotor.set(ControlMode.PercentOutput, -1);
    } else if(wantActiveBalanceLeft) {
      conveyorBottomRightMotor.set(ControlMode.PercentOutput, 1);
    } else {
      conveyorBottomRightMotor.set(ControlMode.PercentOutput, 0);
      conveyorBottomLeftMotor.set(ControlMode.PercentOutput, 0);
    }
  }



}
