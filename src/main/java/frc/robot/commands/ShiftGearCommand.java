
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ShiftGearCommand extends CommandBase {
  private final DriveTrainSubsystem m_driveTrainSubsystem;
  private boolean isHighGear;

  /**
   * Creates a new ArmToggle.
   *
   * @param subsystem The subsystem used by this command.
   */

  public ShiftGearCommand(DriveTrainSubsystem driveTrain, boolean highGear) {
      m_driveTrainSubsystem = driveTrain;
      isHighGear = highGear;
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(driveTrain);    
  }
  @Override
  public void initialize() {
    m_driveTrainSubsystem.setHighGear(isHighGear);
  }

  @Override
  public boolean isFinished(){
      return true;
  }

}