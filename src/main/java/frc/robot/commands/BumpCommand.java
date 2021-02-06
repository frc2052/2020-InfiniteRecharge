
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
  private boolean doBump = false;

  public BumpCommand(DriveTrainSubsystem drive) {
      driveTrain = drive;
      addRequirements(driveTrain);        
  }

  @Override
  public void initialize() {
    doBump = SmartDashboard.getBoolean(Constants.SmartDashboardStrings.kAutoBumpString, false);
    bumpTimer = new Timer();
    bumpTimer.start();
  }

  @Override
  public void execute() {
    if(doBump) {
      driveTrain.arcadeDrive(.75, 0);
    }
    super.execute();
  }

  @Override
  public void end(boolean interrupted) {
      driveTrain.arcadeDrive(0,0);
      bumpTimer.stop();
  }

  @Override
  public boolean isFinished() {
    System.out.println("BumpTimer: " + bumpTimer.get());
    return !doBump || (bumpTimer.get() >= .5);
  }

}