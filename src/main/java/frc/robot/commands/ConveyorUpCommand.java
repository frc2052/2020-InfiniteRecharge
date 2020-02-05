package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ConveyorSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class ConveyorUpCommand extends CommandBase {
  private final ConveyorSubsystem m_subsystem;

  /**
   * Creates a new ConveyorUpCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ConveyorUpCommand(ConveyorSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    
    addRequirements(m_subsystem);
  } 

  @Override
  public void initialize(){
    m_subsystem.lifterUp();
  }

  @Override
  public boolean isFinished(){
    return true;
  }

}