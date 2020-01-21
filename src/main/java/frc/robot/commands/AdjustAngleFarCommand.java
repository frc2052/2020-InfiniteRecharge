/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.HoodSubsystem.anglePresetEnum;

public class AdjustAngleFarCommand extends CommandBase {
  private HoodSubsystem m_HoodSubsystem;
  /**
   * Creates a new AdjustAngleMiddleCommand.
   */
  public AdjustAngleFarCommand(HoodSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_HoodSubsystem = subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_HoodSubsystem.setTarget(anglePresetEnum.FAR);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_HoodSubsystem.getHeightInches() > Constants.Shooter.kFarAnglePosition - 2);
  }
}
