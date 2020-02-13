/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.IShooterControls;
import frc.robot.auto.AutoShooterControls;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.Timer;

public class AutoShooterCommand extends MegaShooterCommand {
  private Timer timer = new Timer();
  private boolean m_IsFinished = false;
  private AutoShooterControls autoControls;

  public AutoShooterCommand(ShooterSubsystem shooter, VisionSubsystem vision, HoodSubsystem hood, TurretSubsystem turret, ConveyorSubsystem conveyor, AutoShooterControls controls) {
    super(shooter, vision, hood, turret, conveyor, controls);
    autoControls = controls;
  }
  
  @Override
  public void initialize() {
    m_IsFinished = false;
    timer.reset();
    super.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    super.execute();
    if(super.getIsReady() && timer.get() == 0) {
      timer.start();
    } 
    if(timer.get() > 4) {
      m_IsFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    autoControls.setShootPressed(false);
    super.end(interrupted);
    m_IsFinished = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_IsFinished;
  }
}
