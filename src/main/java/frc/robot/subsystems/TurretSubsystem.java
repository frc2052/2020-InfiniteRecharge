/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {
  private TalonSRX turretMotor = new TalonSRX(Constants.Turret.kTurretMotorID);
  private int currentPos = 0;
  private double motorPower = 0;
 
  public TurretSubsystem() {
    // TODO reset Talon to factory defaults
    turretMotor.setNeutralMode(NeutralMode.Brake);
    turretMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);

  }
  public void turnTurret(double power){
    motorPower = power;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // TODO do we need to convert this to degrees?
    currentPos = turretMotor.getSelectedSensorPosition();
    if (currentPos > Constants.Turret.kTurretMaxRight && motorPower > 0){
      motorPower = 0;
    } else if (currentPos < Constants.Turret.kTurretMinLeft && motorPower < 0){
      motorPower = 0;
    }
    turretMotor.set(ControlMode.PercentOutput, motorPower);
  }
  
}

