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

}
