/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class ManualSpinUpCommand extends CommandBase {
  private ShooterSubsystem m_ShooterSubsystem;

  /**
   * Creates a new ManualSpinUpCommand.
   */
  //TODO: is this command even needed anymore with the mega shooter
  public ManualSpinUpCommand(ShooterSubsystem shooter) {
    addRequirements(shooter);
    m_ShooterSubsystem = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Shooter Execute");
    m_ShooterSubsystem.testShooterPct(.6);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Shooter End");
    m_ShooterSubsystem.testShooterPct(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
