
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
  private final IntakeSubsystem intake;

  public IntakeCommand(IntakeSubsystem intake) {
      this.intake = intake;

      addRequirements(intake);    
  }
  @Override
  public void initialize() {
    intake.armDown();
    intake.intakeIn();
  }

  @Override
  public boolean isFinished(){
      return true;
  }
}