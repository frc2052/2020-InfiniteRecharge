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
  private TalonSRX turretMotor;
  private int currentPos = 0;
  private double motorPower = 0;
  
  private boolean isLinedUp;
 
  public TurretSubsystem() {
    turretMotor = new TalonSRX(Constants.Motors.kTurretMotorID);
    turretMotor.configFactoryDefault();
    turretMotor.setNeutralMode(NeutralMode.Brake);
    turretMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    turretMotor.setSelectedSensorPosition(0, 0, 10);
  }

  public void turnTurret(double power){
    //motorPower = power;
    turretMotor.set(ControlMode.PercentOutput, power);
  }

  @Override
  public void periodic() {
    // currentPos = turretMotor.getSelectedSensorPosition();
    // if (currentPos > Constants.Turret.kTurretMaxRight && motorPower > 0){
    //   motorPower = 0;
    // } else if (currentPos < Constants.Turret.kTurretMinLeft && motorPower < 0){
    //   motorPower = 0;
    // }
    // turretMotor.set(ControlMode.PercentOutput, motorPower);

    if(turretMotor.getSelectedSensorVelocity() < 0 && turretMotor.getSelectedSensorPosition() < Constants.Turret.kTurretMinEncoderPos) {
      turretMotor.set(ControlMode.PercentOutput, 0);
    } else if(turretMotor.getSelectedSensorVelocity() > 0 && turretMotor.getSelectedSensorPosition() > Constants.Turret.kTurretMaxEncoderPos) {
      turretMotor.set(ControlMode.PercentOutput, 0);
    }
  }
  
  // takes angle and drives until it gets to angle
  public void driveToPos(double angle) {
    isLinedUp = false;
    //System.out.println(turretMotor.getSelectedSensorPosition());
    printEncoderPos();
    //System.out.println("****************** TARGET ANGLE: " + angle);
    if(angle < 0 && turretMotor.getSelectedSensorPosition() < Constants.Turret.kTurretMinEncoderPos) {
      //too far to the negative can't keep going
      turretMotor.set(ControlMode.PercentOutput, 0);
    } else if (angle > 0 && turretMotor.getSelectedSensorPosition() > Constants.Turret.kTurretMaxEncoderPos) {
      //too far to the positive can't keep going
      turretMotor.set(ControlMode.PercentOutput, 0);
    } else {
      if(Math.abs(angle) < 0.2) {
        turretMotor.set(ControlMode.PercentOutput, 0);
        isLinedUp = true;
      } else {
        if (Math.abs(angle) >= 20) {
          isLinedUp = false;
          if(angle > 0) {
            turretMotor.set(ControlMode.PercentOutput, .75);
          } else {
            turretMotor.set(ControlMode.PercentOutput, -.75);
          }
        } else if (Math.abs(angle) < 20) {
          isLinedUp = false;
          turretMotor.set(ControlMode.PercentOutput, (angle * 0.05));
        }
      }
    }
  }

  public boolean getIsOnTarget() {
    return isLinedUp;
  }

  public double getTurretDegree() {
    double ticks = turretMotor.getSelectedSensorPosition(0);
    return ticks / Constants.Turret.kTicksPerDegree;
  }

  public void printEncoderPos() {
    System.out.println("******************************* TURRET ENCODER POS" + turretMotor.getSelectedSensorPosition());
  }
}

