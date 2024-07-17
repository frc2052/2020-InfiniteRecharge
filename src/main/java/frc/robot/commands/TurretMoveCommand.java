// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

public class TurretMoveCommand extends Command {
  private final TurretSubsystem turret;
  private double angle;

  public TurretMoveCommand(double angle, TurretSubsystem turret) {
    this.turret = turret;
    if(angle != -1){
      this.angle = angle;
    } else {
      this.angle = angle;
    }

    addRequirements(turret);
  }

  @Override
  public void initialize() {
    turret.driveToPos(angle);
    System.out.println("driving to pos" + angle);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return turret.getIsOnTarget();
  }
}
