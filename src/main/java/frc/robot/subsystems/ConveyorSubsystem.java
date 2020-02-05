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
  private VictorSPX conveyorBottomLeft;
  private VictorSPX conveyorBottomRight;
  private VictorSPX lifterFront;
  private VictorSPX lifterBack;
  
  /**
   * Creates a new Conveyor.
   */
  public ConveyorSubsystem() {
     
    conveyorBottomLeft = new VictorSPX(Constants.ConveyorSubsystem.kConveyorMotorBottemLeftID);
    conveyorBottomRight = new VictorSPX(Constants.ConveyorSubsystem.kConveyorMotorBottemRightID);
    lifterFront = new VictorSPX(Constants.ConveyorSubsystem.kLifterFrontID);
    lifterBack = new VictorSPX(Constants.ConveyorSubsystem.kLifterBackID);  
    lifterBack.configFactoryDefault();
    lifterFront.configFactoryDefault();
    conveyorBottomLeft.configFactoryDefault();
    conveyorBottomRight.configFactoryDefault();
  
    lifterBack.setInverted(Constants.ConveyorSubsystem.klifterdirection);
    lifterFront.setInverted(!Constants.ConveyorSubsystem.klifterdirection);
    conveyorBottomLeft.setInverted(Constants.ConveyorSubsystem.kconveyordirection);
    conveyorBottomRight.setInverted(!Constants.ConveyorSubsystem.kconveyordirection);
    lifterBack.setNeutralMode(NeutralMode.Coast);
    lifterFront.setNeutralMode(NeutralMode.Coast);
    conveyorBottomLeft.setNeutralMode(NeutralMode.Coast);
    conveyorBottomRight.setNeutralMode(NeutralMode.Coast);
     
  }
  
  public void lifterUp (){
    conveyorBottomLeft.set(ControlMode.PercentOutput, Constants.ConveyorSubsystem.kConveyorSpeed);
    conveyorBottomRight.set(ControlMode.PercentOutput, Constants.ConveyorSubsystem.kConveyorSpeed);
    lifterBack.set(ControlMode.PercentOutput, Constants.ConveyorSubsystem.kConveyorSpeed);
    lifterFront.set(ControlMode.PercentOutput, Constants.ConveyorSubsystem.kConveyorSpeed);
  }
  public void lifterDown (){
    conveyorBottomRight.set(ControlMode.PercentOutput, -Constants.ConveyorSubsystem.kConveyorSpeed);
    conveyorBottomLeft.set(ControlMode.PercentOutput, -Constants.ConveyorSubsystem.kConveyorSpeed);
    lifterBack.set(ControlMode.PercentOutput, -Constants.ConveyorSubsystem.kConveyorSpeed);
    lifterFront.set(ControlMode.PercentOutput, -Constants.ConveyorSubsystem.kConveyorSpeed);
  }
  public void lifterStop (){
    conveyorBottomRight.set(ControlMode.PercentOutput, 0);
    conveyorBottomLeft.set(ControlMode.PercentOutput, 0);
    lifterBack.set(ControlMode.PercentOutput, 0);
    lifterFront.set(ControlMode.PercentOutput, 0);
  }
}
