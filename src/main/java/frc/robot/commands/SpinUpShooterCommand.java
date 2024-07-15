/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class SpinUpShooterCommand extends Command {
  private ShooterSubsystem shooter;

  public SpinUpShooterCommand(ShooterSubsystem shooter) {
    this.shooter = shooter;

    addRequirements(shooter);
  }

  @Override
  public void execute() {
    shooter.setShooterPct(1);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setShooterPct(0);
  }
  
  @Override
  public boolean isFinished() {
    return false;
  }
}
