
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class ArmDownCommand extends CommandBase {
  private final IntakeSubsystem m_intakeSubsystem;

  /**
   * Creates a new ArmToggle.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArmDownCommand(IntakeSubsystem subsystem) {
      m_intakeSubsystem = subsystem;
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(subsystem);    
  }
  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    m_intakeSubsystem.armDown();

  }

  @Override
  public boolean isFinished(){
      return true;
  }

}