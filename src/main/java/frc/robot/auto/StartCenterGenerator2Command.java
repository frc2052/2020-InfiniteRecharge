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

public class StartCenterGenerator2Command extends SequentialCommandGroup {
  public TrajectoryFactory trajectoryFactory = new TrajectoryFactory();

  public StartCenterGenerator2Command(DriveTrainSubsystem driveTrain, ShooterSubsystem shooter, IntakeSubsystem intake, VisionSubsystem vision, HoodSubsystem hood, TurretSubsystem turret, ConveyorSubsystem conveyor, Double delayTime, AutoShooterControls controls) {
    //this.addCommands(new AutoHoodTrimCommand(-50));
    this.addCommands(new BumpCommand(driveTrain));
    this.addCommands(new WaitCommand(delayTime));
    //this.addCommands(new AutoControlsCommand(controls, true, false));
    //this.addCommands(new AutoShooterCommand(shooter, vision, hood, turret, conveyor, controls, 0, 3));
    AutoReadyCommand ready = new AutoReadyCommand(shooter, vision, hood, turret, conveyor, controls, 0);
    Command driveToGenerator2 = trajectoryFactory.getRamseteCommand(driveTrain, DrivePathEnum.CenterLineToGen2);
    Command gen2Backup = trajectoryFactory.getRamseteCommand(driveTrain, DrivePathEnum.Gen2Backup);
    SequentialCommandGroup drivePath = new SequentialCommandGroup(driveToGenerator2, new WaitCommand(0.5), gen2Backup);
    ArmDownCommand intakeCmd = new ArmDownCommand(intake);
    ParallelCommandGroup intakeDriveToGenerator = new ParallelCommandGroup(intakeCmd, drivePath);
    ParallelDeadlineGroup driveIntakeReady = new ParallelDeadlineGroup(intakeDriveToGenerator, ready);
    this.addCommands(driveIntakeReady);
    ArmUpCommand armUp = new ArmUpCommand(intake);
    OuterIntakeStopCommand stopIntake = new OuterIntakeStopCommand(intake);
    SequentialCommandGroup stopIntakeCmds = new SequentialCommandGroup(armUp, stopIntake);
    this.addCommands(stopIntakeCmds);
    this.addCommands(new AutoControlsCommand(controls, true,false));
    this.addCommands(new AutoShooterCommand(shooter, vision, hood, turret, conveyor, controls, 0, 8));
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    SmartDashboard.putNumber(Constants.SmartDashboardStrings.kTurretTrim, 0); //this will run even if our auto ends early-put trim back to 0 so our teleop isnt affected
    SmartDashboard.putNumber(Constants.SmartDashboardStrings.kHoodTrim, 0);
  }
}
