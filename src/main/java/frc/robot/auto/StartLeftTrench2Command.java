/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    this.addCommands(new AutoTurretTrimCommand(-2.5));
//    this.addCommands(new AutoHoodTrimCommand(-50));
      this.addCommands(new BumpCommand(driveTrain));
      this.addCommands(new WaitCommand(delayTime));
      Command driveToMidTrench = trajectoryFactory.getRamseteCommand(driveTrain, DrivePathEnum.StartLeftTrench2);
      ArmDownCommand intakeCmd = new ArmDownCommand(intake);
      ParallelCommandGroup intakeDrive2Balls = new ParallelCommandGroup(intakeCmd, driveToMidTrench);
      this.addCommands(intakeDrive2Balls);
      this.addCommands(new WaitCommand(0.5));
      AutoReadyCommand ready = new AutoReadyCommand(shooter, vision, hood, turret, conveyor, controls, 0);
      ArmUpCommand armUp = new ArmUpCommand(intake);
      OuterIntakeStopCommand stopIntake = new OuterIntakeStopCommand(intake);
      SequentialCommandGroup stopIntakeCmds = new SequentialCommandGroup(armUp, stopIntake);
      Command driveToShootSpot = trajectoryFactory.getRamseteCommand(driveTrain, DrivePathEnum.Trench2ToShoot);
      Command backUpToShoot = trajectoryFactory.getRamseteCommand(driveTrain, DrivePathEnum.LeftTrenchBackup);
      SequentialCommandGroup drivePath = new SequentialCommandGroup(driveToShootSpot, backUpToShoot);
      ParallelDeadlineGroup driveReady = new ParallelDeadlineGroup(drivePath, stopIntakeCmds, ready);
      this.addCommands(driveReady);
      this.addCommands(new AutoControlsCommand(controls, true, true));
      this.addCommands(new AutoShooterCommand(shooter, vision, hood, turret, conveyor, controls, 0, 7));
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    SmartDashboard.putNumber(Constants.SmartDashboardStrings.kTurretTrim, 0); //this will run even if our auto ends early-put trim back to 0 so our teleop isnt affected
    SmartDashboard.putNumber(Constants.SmartDashboardStrings.kHoodTrim, 0);
  }
}
