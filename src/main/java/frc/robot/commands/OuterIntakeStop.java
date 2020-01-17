import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class OuterIntakeStop extends CommandBase {
    private final IntakeSubsystem m_intakeSubsystem;

    /**
     * Creates a new OuterIntakeStop.
     *
     * @param subsystem The subsystem used by this command.
     */
    public OuterIntakeStop(IntakeSubsystem subsystem) {
        m_intakeSubsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }
    @Override
        public void initialize() {
            m_intakeSubsystem.intakeStop ();
        }
}