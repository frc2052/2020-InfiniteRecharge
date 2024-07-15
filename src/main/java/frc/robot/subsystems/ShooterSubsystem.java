/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  private TalonFX shooterMotor;
  private double lastShootPct = 0;
  
  /**
   * Creates a new FalconShooterSubsystem.
   */
  public ShooterSubsystem() {
    shooterMotor = new TalonFX(Constants.Motors.kFalconShooterMotorId);
    shooterMotor.setNeutralMode(NeutralModeValue.Coast);

    Slot0Configs config = new Slot0Configs();
    config.kP = 1.3;
    config.kI = 0;
    config.kD = 0.5;
    shooterMotor.getConfigurator().apply(config, 0.5);
  }

  public double getSpeedPct() {
    return lastShootPct;
  }

  public void setShooterPct(double pct) {
    shooterMotor.set(pct);
    lastShootPct = pct;
  }
}
