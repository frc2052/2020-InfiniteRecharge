// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

public class TurretMoveCommand extends Command {
  private final TurretSubsystem turret;
  private final double power;

  public TurretMoveCommand(double power, TurretSubsystem turret) {
    this.power = power;
    this.turret = turret;
    addRequirements(turret);
  }

  @Override
  public void initialize() {
    turret.turnTurret(power);
  }

  @Override
  public void execute() {
    System.out.println("turret pos: " + turret.getEncoderPos());
  }

  @Override
  public void end(boolean interrupted) {
    turret.turnTurret(0);}

  @Override
  public boolean isFinished() {
    return false;
  }
}
