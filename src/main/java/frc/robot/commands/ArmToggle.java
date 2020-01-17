import edu.wpi.first.wpilibj.templates.commandbased.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class ArmToggle extends CommandBase {
    private final IntakeSubsystem m_intakeSubsystem;

    /**
     * Creates a new ArmToggle.
     *
     * @param subsystem The subsystem used by this command.
     */
    public ArmToggle(IntakeSubsystem subsystem) {
        m_intakeSubsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
        public void initialize() {
            m_intakeSubsystem.armToggle();
        }
    }
}