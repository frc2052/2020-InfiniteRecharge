
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
  private final IntakeSubsystem intake;
  private final ConveyorSubsystem conveyor;

  public IntakeCommand(IntakeSubsystem intake, ConveyorSubsystem conveyor) {
      this.intake = intake;
      this.conveyor = conveyor;

      addRequirements(intake, conveyor);    
  }
  @Override
  public void initialize() {
    intake.intakeIn();
    conveyor.setWantBottomFeed(true);
  }

  @Override
  public boolean isFinished(){
      return true;
  }
}