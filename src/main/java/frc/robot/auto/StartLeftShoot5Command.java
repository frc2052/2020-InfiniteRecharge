/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.*;
import frc.robot.Constants;
import frc.robot.auto.TrajectoryFactory.DrivePathEnum;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class StartLeftShoot5Command extends SequentialCommandGroup {
  public TrajectoryFactory trajectoryFactory = new TrajectoryFactory();

  public StartLeftShoot5Command(DriveTrainSubsystem driveTrain, ShooterSubsystem shooter, IntakeSubsystem intake, VisionSubsystem vision, HoodSubsystem hood, TurretSubsystem turret, ConveyorSubsystem conveyor, Double delayTime, AutoShooterControls controls) {
    this.addCommands(new BumpCommand(driveTrain));
    this.addCommands(new WaitCommand(delayTime));
    this.addCommands(new AutoShooterCommand(shooter, vision, hood, turret, conveyor, controls, Constants.Autonomous.kLeftInitTargetTicks, 3));
    ArmDownCommand intakeCmd = new ArmDownCommand(intake);
    Command ramsete = trajectoryFactory.getRamseteCommand(driveTrain, DrivePathEnum.StartLeftGenerator5);
    ArmDownCommand armDownCmd = new ArmDownCommand(intake);
    ParallelCommandGroup par1 = new ParallelCommandGroup(intakeCmd, ramsete, armDownCmd);
    this.addCommands(par1);
    this.addCommands(new OuterIntakeStopCommand(intake));
    this.addCommands(new AutoShooterCommand(shooter, vision, hood, turret, conveyor, controls, 0, 5));
  }
}
