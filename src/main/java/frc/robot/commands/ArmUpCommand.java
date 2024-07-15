package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class ArmUpCommand extends Command {
    private final IntakeSubsystem intake;
    
    public ArmUpCommand(IntakeSubsystem intake) {
        this.intake = intake;

        addRequirements(intake);    
    }

    @Override
    public void initialize() {
        intake.armUp();
        intake.intakeStop();
    }

    @Override
    public boolean isFinished(){
        return true;
    }


}