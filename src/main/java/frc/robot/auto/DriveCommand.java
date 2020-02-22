/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.TrajectoryFactory.DrivePathEnum;
import frc.robot.subsystems.DriveTrainSubsystem;

public class DriveCommand extends SequentialCommandGroup {
  public TrajectoryFactory trajectoryFactory = new TrajectoryFactory();

  public DriveCommand(DriveTrainSubsystem driveTrain) {
    this.addCommands(trajectoryFactory.getRamseteCommand(driveTrain, DrivePathEnum.Drive));
  }
}
