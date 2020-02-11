/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.*;
import frc.robot.auto.TrajectoryFactory.DrivePathEnum;
import frc.robot.subsystems.*;

public class StartRightTrench3Command extends SequentialCommandGroup {
  public TrajectoryFactory trajectoryFactory = new TrajectoryFactory();

  public StartRightTrench3Command(DriveTrainSubsystem driveTrain, ShooterSubsystem shooter, IntakeSubsystem intake, VisionSubsystem vision, HoodSubsystem hood, TurretSubsystem turret, ConveyorSubsystem conveyor, Double delayTime) {
    
    this.addCommands(new BumpCommand(driveTrain));
      
    this.addCommands(new WaitCommand(delayTime));
    this.addCommands(new AutoShooterCommand(shooter, vision, hood, turret, conveyor));    
    ArmDownCommand intakeCmd = new ArmDownCommand(intake);
    RamseteCommand ramsete = trajectoryFactory.getRamseteCommand(driveTrain, DrivePathEnum.StartRightTrench3Ball);
    ParallelCommandGroup par1 = new ParallelCommandGroup(intakeCmd, ramsete);
    this.addCommands(par1);
    this.addCommands(new OuterIntakeStopCommand(intake));
    this.addCommands(new AutoShooterCommand(shooter, vision, hood, turret, conveyor));
  }

}
