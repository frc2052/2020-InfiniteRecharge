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
  private final VictorSPX ConveyorBottomLeft = new VictorSPX(Constants.ConveyorSubsystem.kConveyorMotorBottemLeftID);  
  private final VictorSPX ConveyorBottomRight = new VictorSPX(Constants.ConveyorSubsystem.kConveyorMotorBottemRightID);
  private final VictorSPX LifterFront = new VictorSPX(Constants.ConveyorSubsystem.kLifterFrontID);
  private final VictorSPX LifterBack = new VictorSPX(Constants.ConveyorSubsystem.kLifterBackID);
public void declarations(){
  LifterBack.setInverted(Constants.ConveyorSubsystem.klifterdirection);
  LifterFront.setInverted(!Constants.ConveyorSubsystem.klifterdirection);
  ConveyorBottomLeft.setInverted(Constants.ConveyorSubsystem.kconveyordirection);
  ConveyorBottomRight.setInverted(!Constants.ConveyorSubsystem.kconveyordirection);
  LifterBack.setNeutralMode(NeutralMode.Coast);
  LifterFront.setNeutralMode(NeutralMode.Coast);
  ConveyorBottomLeft.setNeutralMode(NeutralMode.Coast);
  ConveyorBottomRight.setNeutralMode(NeutralMode.Coast);
}
  
public void LifterUp (){
  ConveyorBottomLeft.set(ControlMode.PercentOutput, 50);
  ConveyorBottomRight.set(ControlMode.PercentOutput, 50);
  LifterBack.set(ControlMode.PercentOutput, 50);
  LifterFront.set(ControlMode.PercentOutput, 50);
}
public void LifterDown (){
  ConveyorBottomRight.set(ControlMode.PercentOutput, -50);
  ConveyorBottomLeft.set(ControlMode.PercentOutput, -50);
  LifterBack.set(ControlMode.PercentOutput, -50);
  LifterFront.set(ControlMode.PercentOutput, -50);
}
public void LifterStop (){
  ConveyorBottomRight.set(ControlMode.PercentOutput, 0);
  ConveyorBottomLeft.set(ControlMode.PercentOutput, 0);
  LifterBack.set(ControlMode.PercentOutput, 0);
  LifterFront.set(ControlMode.PercentOutput, 0);
}


  


/**
   * Creates a new Convayer.
   */
   public ConveyorSubsystem() {}
}
