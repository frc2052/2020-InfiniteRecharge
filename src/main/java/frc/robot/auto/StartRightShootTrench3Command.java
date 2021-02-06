/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.*;
import frc.robot.auto.TrajectoryFactory.DrivePathEnum;
import frc.robot.commands.*;

public class StartRightShootTrench3Command extends SequentialCommandGroup {
  public TrajectoryFactory trajectoryFactory = new TrajectoryFactory();

  public StartRightShootTrench3Command(DriveTrainSubsystem driveTrain, ShooterSubsystem shooter, IntakeSubsystem intake, VisionSubsystem vision, HoodSubsystem hood, TurretSubsystem turret, ConveyorSubsystem conveyor, Double delayTime, AutoShooterControls controls) {
    this.addCommands(new AutoTurretTrimCommand(-2));
    AutoReadyCommand ready = new AutoReadyCommand(shooter, vision, hood, turret, conveyor, controls, 0);
    AutoShooterCommand shoot = new AutoShooterCommand(shooter, vision, hood, turret, conveyor, controls, 0, 2.75);
    ParallelDeadlineGroup getReady = new ParallelDeadlineGroup(new WaitCommand(1), ready);
    //this.addCommands(getReady);
    this.addCommands(new AutoControlsCommand(controls, true, true));
    this.addCommands(shoot);
    Command driveBack = trajectoryFactory.getRamseteCommand(driveTrain, DrivePathEnum.LineToTrenchBack);
    ArmDownCommand intakeCmd = new ArmDownCommand(intake);
    ParallelCommandGroup driveIntake = new ParallelCommandGroup(driveBack, intakeCmd);
    this.addCommands(driveIntake);
    this.addCommands(new ArmUpCommand(intake));
    this.addCommands(new OuterIntakeStopCommand(intake));
    AutoReadyCommand ready2 = new AutoReadyCommand(shooter, vision, hood, turret, conveyor, controls, 0);
    Command driveToLine = trajectoryFactory.getRamseteCommand(driveTrain, DrivePathEnum.TrenchBackToLine);
    ParallelDeadlineGroup driveReady = new ParallelDeadlineGroup(driveToLine, ready2);
    this.addCommands(driveReady);
    this.addCommands(new AutoControlsCommand(controls, true, true));
    this.addCommands(new AutoShooterCommand(shooter, vision, hood, turret, conveyor, controls, 0, 4));
  }
}
