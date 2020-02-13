/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;

import frc.robot.commands.*;
import frc.robot.auto.TrajectoryFactory.DrivePathEnum;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.*;

public class CenterShootDriveParkCommand extends SequentialCommandGroup {
  public TrajectoryFactory trajectoryFactory = new TrajectoryFactory();

  public CenterShootDriveParkCommand(DriveTrainSubsystem driveTrain, ShooterSubsystem shooter, VisionSubsystem vision, HoodSubsystem hood, TurretSubsystem turret, ConveyorSubsystem conveyor, Double delayTime, AutoShooterControls controls) {    
    this.addCommands(new BumpCommand(driveTrain));
    this.addCommands(new WaitCommand(delayTime));
    this.addCommands(new AutoShooterCommand(shooter, vision, hood, turret, conveyor, controls));
    this.addCommands(trajectoryFactory.getRamseteCommand(driveTrain, DrivePathEnum.StartCenterDriveBackPark));
  }
}
