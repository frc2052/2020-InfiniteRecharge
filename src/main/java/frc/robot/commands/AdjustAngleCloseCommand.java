package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.anglePresetEnum;

public class AdjustAngleCloseCommand extends CommandBase {
  private ShooterSubsystem m_ShooterSubsystem;

  public AdjustAngleCloseCommand(ShooterSubsystem subsystem) {
    m_ShooterSubsystem = subsystem;
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
    m_ShooterSubsystem.setTarget(anglePresetEnum.CLOSE);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_ShooterSubsystem.getHeightInches() > Constants.Shooter.kCloseAnglePosition -2);
  }
}
