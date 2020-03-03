/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class AutoFeed extends SequentialCommandGroup {
  public TrajectoryFactory trajectoryFactory = new TrajectoryFactory();

  public AutoFeed(DriveTrainSubsystem driveTrain, ShooterSubsystem shooter, IntakeSubsystem intake, VisionSubsystem vision, HoodSubsystem hood, TurretSubsystem turret, ConveyorSubsystem conveyor, AutoShooterControls controls, Double delayTime) {
    this.addCommands(new WaitCommand(delayTime));
    this.addCommands(new ArmDownCommand(intake));
    this.addCommands(new AutoShooterCommand(shooter, vision, hood, turret, conveyor, controls, 6400, 10));
    this.addCommands(new OuterIntakeStopCommand(intake));
    //TODO add a drive path? note: we already have an enum, although it is default.
  }

}
