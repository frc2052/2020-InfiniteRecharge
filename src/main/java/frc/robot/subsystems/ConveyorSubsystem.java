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
  private boolean wantBottomFeed = false;
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

  public void setWantUp(boolean isPressed) {
    wantConveyorUp = isPressed;
  }

  public void setWantBottomFeed(boolean isPressed) {
    wantBottomFeed = isPressed;
  }

  @Override
  public void periodic() {

    if(wantConveyorUp) {
      lifterMotor.set(ControlMode.PercentOutput, -1);
    } else if (!wantConveyorUp){
      lifterMotor.set(ControlMode.PercentOutput, 0);
    }

    if(wantConveyorUp || wantBottomFeed) {
        conveyorBottomRightMotor.set(ControlMode.PercentOutput, .75);
        conveyorBottomLeftMotor.set(ControlMode.PercentOutput, -1);
    } else {
      conveyorBottomRightMotor.set(ControlMode.PercentOutput, 0);
      conveyorBottomLeftMotor.set(ControlMode.PercentOutput, 0);
    }
  }



}
