
package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveTrain;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class BumpCommand extends CommandBase {
    private Timer bumpTimer;
  /**
   * Checks is bump is true bump the other robots
   *
   * @param subsystem The subsystem used by this command.
   */

  private DriveTrainSubsystem driveTrain;

  public BumpCommand(DriveTrainSubsystem drive) {
    if(SmartDashboard.getBoolean(Constants.SmartDashboard.kAutoBumpString, true)) {
        driveTrain = drive;
        addRequirements(driveTrain);
        bumpTimer = new Timer();
        
    }
  }

  @Override
  public void initialize() {
    if(SmartDashboard.getBoolean(Constants.SmartDashboard.kAutoBumpString, true)) {
      driveTrain.arcadeDrive(.5, 0);
      bumpTimer.start();
      
    }
  }

  @Override
  public boolean isFinished() {
      return (bumpTimer.get() >= 1.0);
  }

}