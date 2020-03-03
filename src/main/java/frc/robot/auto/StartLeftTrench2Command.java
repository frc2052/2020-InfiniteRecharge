/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.*;
import frc.robot.Constants;
import frc.robot.auto.TrajectoryFactory;
import frc.robot.auto.TrajectoryFactory.DrivePathEnum;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

import frc.robot.subsystems.*;


public class StartLeftTrench2Command extends SequentialCommandGroup {
  public TrajectoryFactory trajectoryFactory = new TrajectoryFactory();

  public StartLeftTrench2Command(DriveTrainSubsystem driveTrain, ShooterSubsystem shooter, IntakeSubsystem intake, VisionSubsystem vision, HoodSubsystem hood, TurretSubsystem turret, ConveyorSubsystem conveyor, Double delayTime, AutoShooterControls controls) {
      this.addCommands(new BumpCommand(driveTrain));
      this.addCommands(new WaitCommand(delayTime));
      this.addCommands(new AutoControlsCommand(controls, false, true));
      Command driveToMidTrench = trajectoryFactory.getRamseteCommand(driveTrain, DrivePathEnum.StartLeftTrench2);
      ArmDownCommand intakeCmd = new ArmDownCommand(intake);
      ParallelCommandGroup intakeDrive2Balls = new ParallelCommandGroup(intakeCmd, driveToMidTrench);
      this.addCommands(intakeDrive2Balls);
      AutoReadyCommand ready = new AutoReadyCommand(shooter, vision, hood, turret, conveyor, controls, 0);
      Command driveToShootSpot = trajectoryFactory.getRamseteCommand(driveTrain, DrivePathEnum.Trench2ToShoot);
      ParallelDeadlineGroup driveReady = new ParallelDeadlineGroup(driveToShootSpot, ready);
      this.addCommands(driveReady);
      this.addCommands(new AutoControlsCommand(controls, true, true));
      this.addCommands(new AutoShooterCommand(shooter, vision, hood, turret, conveyor, controls, 0, 5));
      this.addCommands(new ArmUpCommand(intake));
      this.addCommands(new OuterIntakeStopCommand(intake));
      
  }
}
