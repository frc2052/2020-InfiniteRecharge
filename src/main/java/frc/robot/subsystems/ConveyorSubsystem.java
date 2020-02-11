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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ConveyorSubsystem extends SubsystemBase {
  private VictorSPX conveyorBottomLeftMotor;
  private VictorSPX conveyorBottomRightMotor;
  private VictorSPX lifterMotor;
  
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
  
  public void lifterUp (){
    conveyorBottomLeftMotor.set(ControlMode.PercentOutput, -.4);
    conveyorBottomRightMotor.set(ControlMode.PercentOutput, -.6);
    lifterMotor.set(ControlMode.PercentOutput, -.4);
  }

  public void lifterDown (){
    conveyorBottomRightMotor.set(ControlMode.PercentOutput, -Constants.ConveyorSubsystem.kConveyorSpeed);
    conveyorBottomLeftMotor.set(ControlMode.PercentOutput, -Constants.ConveyorSubsystem.kConveyorSpeed);
    lifterMotor.set(ControlMode.PercentOutput, -Constants.ConveyorSubsystem.kConveyorSpeed);
  }

  public void lifterStop (){
    conveyorBottomRightMotor.set(ControlMode.PercentOutput, 0);
    conveyorBottomLeftMotor.set(ControlMode.PercentOutput, 0);
    lifterMotor.set(ControlMode.PercentOutput, 0);
  }

  public void preLoad() {
    conveyorBottomLeftMotor.set(ControlMode.PercentOutput, .2);
    conveyorBottomRightMotor.set(ControlMode.PercentOutput, .2);
  }



}
