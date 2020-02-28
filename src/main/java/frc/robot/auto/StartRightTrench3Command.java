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
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.*;
import frc.robot.Constants;
import frc.robot.IShooterControls;
import frc.robot.auto.TrajectoryFactory.DrivePathEnum;
import frc.robot.subsystems.*;

public class StartRightTrench3Command extends SequentialCommandGroup {
  public TrajectoryFactory trajectoryFactory = new TrajectoryFactory();

  public StartRightTrench3Command(DriveTrainSubsystem driveTrain, ShooterSubsystem shooter, IntakeSubsystem intake, VisionSubsystem vision, HoodSubsystem hood, TurretSubsystem turret, ConveyorSubsystem conveyor, Double delayTime, AutoShooterControls controls) {
    this.addCommands(new BumpCommand(driveTrain));
    this.addCommands(new WaitCommand(delayTime));
    this.addCommands(new AutoControlsCommand(controls, false, true));

    AutoReadyCommand ready1 = new AutoReadyCommand(shooter, vision, hood, turret, conveyor, controls, 0);
    Command drive = trajectoryFactory.getRamseteCommand(driveTrain, DrivePathEnum.LineToTrench);
    ParallelDeadlineGroup driveShoot = new ParallelDeadlineGroup(drive, ready1);
    this.addCommands(driveShoot); 
    this.addCommands(new AutoControlsCommand(controls, true, true));
    this.addCommands(new AutoShooterCommand(shooter, vision, hood, turret, conveyor, controls, 0, 3));
    ArmDownCommand intakeCmd = new ArmDownCommand(intake);
    Command ramsete = trajectoryFactory.getRamseteCommand(driveTrain, DrivePathEnum.TrenchFrontToBack);
    ParallelCommandGroup par1 = new ParallelCommandGroup(intakeCmd, ramsete);
    AutoReadyCommand ready2 = new AutoReadyCommand(shooter, vision, hood, turret, conveyor, controls, 0);
    ParallelDeadlineGroup driveShoot2 = new ParallelDeadlineGroup(par1, ready2);
    this.addCommands(driveShoot2);
    this.addCommands(new ArmUpCommand(intake));
    this.addCommands(new OuterIntakeStopCommand(intake));
    this.addCommands(trajectoryFactory.getRamseteCommand(driveTrain, DrivePathEnum.TrenchBackToFront));
    this.addCommands(new AutoControlsCommand(controls, true, true));
    this.addCommands(new AutoShooterCommand(shooter, vision, hood, turret, conveyor, controls, 0, 5));
  }
}
