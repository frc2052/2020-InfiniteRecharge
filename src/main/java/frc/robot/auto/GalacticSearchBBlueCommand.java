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

public class GalacticSearchBBlueCommand extends SequentialCommandGroup {
  public TrajectoryFactory trajectoryFactory = new TrajectoryFactory();

  public GalacticSearchBBlueCommand(DriveTrainSubsystem driveTrain, IntakeSubsystem intake, VisionSubsystem vision) {
    // Command path1 = trajectoryFactory.getRamseteCommand(driveTrain, DrivePathEnum.GalacticSearchBBlue1);
    // this.addCommands(new ShiftGearCommand(driveTrain, true));
    // this.addCommands(path1);
    // this.addCommands(new ShiftGearCommand(driveTrain, false));
    // Command path2 = trajectoryFactory.getRamseteCommand(driveTrain, DrivePathEnum.GalacticSearchBRed2);
    // ArmDownCommand intakeStart = new ArmDownCommand(intake);
    // ParallelCommandGroup driveIntake = new ParallelCommandGroup(path1, intakeStart);
    // this.addCommands(driveIntake);
    // this.addCommands(new ArmUpCommand(intake));

    Command path1 = trajectoryFactory.getRamseteCommand(driveTrain, DrivePathEnum.GalacticSearchBBlue1);
    Command path2 = trajectoryFactory.getRamseteCommand(driveTrain, DrivePathEnum.GalacticSearchBBlue2);
    this.addCommands(path1);
    this.addCommands(path2);
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
  }
}
