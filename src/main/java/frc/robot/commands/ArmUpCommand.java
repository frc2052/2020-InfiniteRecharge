package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ArmUpCommand extends Command {
    private final IntakeSubsystem intake;
    private final ConveyorSubsystem conveyor;
  
    public ArmUpCommand(IntakeSubsystem intake, ConveyorSubsystem conveyor) {
        this.intake = intake;
        this.conveyor = conveyor;
  
        addRequirements(intake, conveyor);    
    }

    @Override
    public void initialize() {
        intake.armUp();
        intake.intakeStop();
        conveyor.setWantBottomFeed(false);
    }

    @Override
    public boolean isFinished(){
        return true;
    }


}