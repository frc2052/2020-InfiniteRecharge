/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.auto.TrajectoryFactory.DrivePathEnum;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class AutoNavBounceCommand extends SequentialCommandGroup {
  public TrajectoryFactory trajectoryFactory = new TrajectoryFactory();

  public AutoNavBounceCommand(DriveTrainSubsystem driveTrain) {
    Command path = trajectoryFactory.getRamseteCommand(driveTrain, DrivePathEnum.AutoNavBounce);
    this.addCommands(path);
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
  }
}
