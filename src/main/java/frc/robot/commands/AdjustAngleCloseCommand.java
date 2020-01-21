package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.HoodSubsystem.anglePresetEnum;

public class AdjustAngleCloseCommand extends CommandBase {
  private HoodSubsystem m_HoodSubsystem;

  public AdjustAngleCloseCommand(HoodSubsystem subsystem) {
    m_HoodSubsystem = subsystem;
  }
  /**
   * Creates a new AdjustAngleCloseCommand.
   */

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_HoodSubsystem.setTarget(anglePresetEnum.CLOSE);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_HoodSubsystem.getHeightInches() > Constants.Shooter.kCloseAnglePosition -2);
  }
}
