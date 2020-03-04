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
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.*;
import frc.robot.Constants;
import frc.robot.IShooterControls;
import frc.robot.Constants.SmartDashboardStrings;
import frc.robot.auto.TrajectoryFactory.DrivePathEnum;
import frc.robot.subsystems.*;

public class StartRightTrench3Command extends SequentialCommandGroup {
  public TrajectoryFactory trajectoryFactory = new TrajectoryFactory();

  public StartRightTrench3Command(DriveTrainSubsystem driveTrain, ShooterSubsystem shooter, IntakeSubsystem intake, VisionSubsystem vision, HoodSubsystem hood, TurretSubsystem turret, ConveyorSubsystem conveyor, Double delayTime, AutoShooterControls controls) {
    this.addCommands(new AutoTurretTrimCommand(-2));
    this.addCommands(new BumpCommand(driveTrain));
    this.addCommands(new WaitCommand(delayTime));
    AutoReadyCommand ready = new AutoReadyCommand(shooter, vision, hood, turret, conveyor, controls, 0);
    Command driveToMidTrench = new PathCommand(driveTrain, DrivePathEnum.RightLineToTrenchMiddle);
    ArmDownCommand intakeCmd = new ArmDownCommand(intake);
    ParallelCommandGroup intakeDrive2Balls = new ParallelCommandGroup(intakeCmd, driveToMidTrench);
    ParallelDeadlineGroup driveReady = new ParallelDeadlineGroup(intakeDrive2Balls, ready);
    this.addCommands(driveReady); 
    this.addCommands(new AutoControlsCommand(controls, true, true));
    this.addCommands(new AutoShooterCommand(shooter, vision, hood, turret, conveyor, controls, 0, 3));
    Command driveToBackTrench = new PathCommand(driveTrain, DrivePathEnum.TrenchMiddleToBack);
    AutoReadyCommand ready2 = new AutoReadyCommand(shooter, vision, hood, turret, conveyor, controls, 0);
    ParallelDeadlineGroup driveReady2 = new ParallelDeadlineGroup(driveToBackTrench, ready2);
    this.addCommands(driveReady2);
    this.addCommands(new ArmUpCommand(intake));
    this.addCommands(new OuterIntakeStopCommand(intake));
    this.addCommands(new AutoControlsCommand(controls, true, true));
    this.addCommands(new AutoShooterCommand(shooter, vision, hood, turret, conveyor, controls, 0, 1.5));
    
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    SmartDashboard.putNumber(Constants.SmartDashboardStrings.kTurretTrim, 0); //this will run even if our auto ends early-put trim back to 0 so our teleop isnt affected
  }
}
