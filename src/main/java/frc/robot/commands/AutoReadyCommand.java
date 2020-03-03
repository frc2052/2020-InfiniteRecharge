/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.auto.AutoShooterControls;
import frc.robot.subsystems.*;

public class AutoReadyCommand extends MegaShooterCommand {
  private AutoShooterControls autoControls;

  private int readyTime = 0;
  private Timer timer = new Timer();


  public AutoReadyCommand(ShooterSubsystem shooter, VisionSubsystem vision, HoodSubsystem hood, TurretSubsystem turret, ConveyorSubsystem conveyor, AutoShooterControls controls, int timeReady) {
    super(shooter, vision, hood, turret, conveyor, controls);
    autoControls = controls;
    readyTime = timeReady;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    autoControls.setReadyPressed(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    super.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
