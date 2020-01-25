/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;

import frc.robot.commands.*;
import frc.robot.commands.DrivePathCommand.DrivePathEnum;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.*;

public class CenterShootDriveParkCommand extends SequentialCommandGroup {

  public CenterShootDriveParkCommand(DriveTrainSubsystem driveTrain, ShooterSubsystem shooter, VisionSubsystem vision, Double delayTime) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.addCommands(new WaitCommand(delayTime));
    this.addCommands(new ShootAllCommand(shooter, vision));
    this.addCommands(new DrivePathCommand(driveTrain, DrivePathEnum.StartCenterDriveBackPark));


  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
