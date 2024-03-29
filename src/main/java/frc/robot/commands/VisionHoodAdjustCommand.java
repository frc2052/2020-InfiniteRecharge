/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class VisionHoodAdjustCommand extends CommandBase {
  private VisionSubsystem vision;
  private HoodSubsystem hood;
  boolean isManual;

  //TODO is this even needed assuming the mega shooter works
  public VisionHoodAdjustCommand(VisionSubsystem visionSubsystem, HoodSubsystem hoodSubsystem, boolean manual) {
    vision = visionSubsystem;
    hood = hoodSubsystem;
    isManual = manual;
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!isManual) {
      
    }
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
