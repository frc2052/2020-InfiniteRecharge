package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ConveyorSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class ConveyorStopCommand extends CommandBase {
  private final ConveyorSubsystem m_subsystem;

  /**
   * Creates a new ConveyorStopCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ConveyorStopCommand(ConveyorSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
        
    addRequirements(subsystem);
  } 

  @Override
  public void initialize(){
    m_subsystem.lifterStop();
  }
  @Override
  public boolean isFinished(){
    return true;
  }
}