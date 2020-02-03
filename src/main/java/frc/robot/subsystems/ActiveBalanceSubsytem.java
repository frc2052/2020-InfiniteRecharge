/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ActiveBalanceSubsytem extends SubsystemBase {
  //TODO: The active balancer only has one motor, confirm if a Talon or Victor with build team
  //TODO: remove final, new up the objects in the constructor
  private final WPI_TalonSRX ActiveBalanceMotor = new WPI_TalonSRX(Constants.ActiveBalance.kActiveBalanceMotorID);
  private final VictorSPX ActiveBalanceMotor2 = new VictorSPX(Constants.ActiveBalance.kActiveBalanceMotor2ID);
 
  public ActiveBalanceSubsytem() {
    //TODO reset motor controllers to factory defaults
    //TODO set motor to break mode
  }
  public void ManualLeft() {
    ActiveBalanceMotor.set(ControlMode.PercentOutput,.2);
    ActiveBalanceMotor2.set(ControlMode.PercentOutput,.2);
  }
  public void ManualRight() {
    ActiveBalanceMotor.set(ControlMode.PercentOutput,-.2);
    ActiveBalanceMotor2.set(ControlMode.PercentOutput,-.2);
  }
  public void ManualStop() {
    ActiveBalanceMotor.set(ControlMode.PercentOutput,0);
    ActiveBalanceMotor2.set(ControlMode.PercentOutput,0);
 }
  
}
