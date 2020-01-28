package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class OuterIntakeStopCommand extends CommandBase {
    private IntakeSubsystem m_intakeSubsystem;

    /**
     * Creates a new OuterIntakeStop.
     *
     */

    public OuterIntakeStopCommand(IntakeSubsystem subsystem) {
        m_intakeSubsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }
    @Override
    public void execute() {
        m_intakeSubsystem.intakeStop ();

    }

    @Override
    public void initialize() {
    }
    @Override
    public boolean isFinished(){
        return true;
    }
}