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

public class ActiveBalanceSubsytem extends SubsystemBase {
  private final VictorSPX activeBalanceMotor;
 
  public ActiveBalanceSubsytem() {
    activeBalanceMotor = new VictorSPX(Constants.Motors.kActiveBalanceMotorID);
    activeBalanceMotor.configFactoryDefault();
    activeBalanceMotor.setNeutralMode(NeutralMode.Brake);
  }
  public void manualLeft() {
    activeBalanceMotor.set(ControlMode.PercentOutput, Constants.ActiveBalanceConstants.kActiveBalanceSpeed);
  }
  public void manualRight() {
    activeBalanceMotor.set(ControlMode.PercentOutput, -Constants.ActiveBalanceConstants.kActiveBalanceSpeed);
  }
  public void manualStop() {
    activeBalanceMotor.set(ControlMode.PercentOutput, 0);
 }
}
