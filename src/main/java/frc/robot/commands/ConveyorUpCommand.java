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
    
    //TODO: this probably needs to be moved to an initalize method
    subsystem.lifterUp();
    addRequirements(subsystem);
  } 

  //need to add a isFinished method that returns true
}