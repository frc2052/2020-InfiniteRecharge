/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.TrajectoryFactory;
import frc.robot.auto.TrajectoryFactory.DrivePathEnum;
import frc.robot.subsystems.DriveTrainSubsystem;

public class PathCommand extends SequentialCommandGroup {
  private DriveTrainSubsystem m_driveTrain;
  private DrivePathEnum m_path;

  TrajectoryFactory trajectoryFactory = new TrajectoryFactory();

  public PathCommand(DriveTrainSubsystem driveTrain, DrivePathEnum path) {
    m_driveTrain = driveTrain;
    m_path = path;
    if (m_driveTrain == null)
    {
        System.err.println("DRIVE TRAIN NULL HERE TOO ^^^^^^^^^^^^^^^^^^^^^^^^^^");
    }
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.addCommands(trajectoryFactory.getRamseteCommand(m_driveTrain, m_path));
    super.initialize();
  }
}
