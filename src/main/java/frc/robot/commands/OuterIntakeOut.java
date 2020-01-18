
package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class OuterIntakeOut extends CommandBase {
    private IntakeSubsystem m_intakeSubsystem;

    /**
     * Creates a new OuterIntakeOut.
     *
     * @param subsystem The subsystem used by this command.
     */
    public OuterIntakeOut(IntakeSubsystem subsystem) {
        m_intakeSubsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);

    }
    @Override
    public void execute() {
        m_intakeSubsystem.intakeOut ();

    }
}