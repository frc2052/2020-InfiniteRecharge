/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.commands.*;
import frc.robot.auto.TrajectoryFactory.DrivePathEnum;
import frc.robot.subsystems.*;

public class StartLeftGenerator3Command extends SequentialCommandGroup {
  public TrajectoryFactory trajectoryFactory = new TrajectoryFactory();

  public StartLeftGenerator3Command(DriveTrainSubsystem driveTrain, ShooterSubsystem shooter, IntakeSubsystem intake, VisionSubsystem vision, HoodSubsystem hood, TurretSubsystem turret, ConveyorSubsystem conveyor, Double delayTime, AutoShooterControls controls) {
	    this.addCommands(new WaitCommand(delayTime));
      this.addCommands(new AutoShooterCommand(shooter, vision, hood, turret, conveyor, controls));
      ArmDownCommand intakeCmd = new ArmDownCommand(intake);
      RamseteCommand ramsete = trajectoryFactory.getRamseteCommand(driveTrain, DrivePathEnum.StartLeftGenerator3);
      ParallelCommandGroup par1 = new ParallelCommandGroup(intakeCmd, ramsete);
      this.addCommands(par1);
      this.addCommands(new OuterIntakeStopCommand(intake));
      this.addCommands(new AutoShooterCommand(shooter, vision, hood, turret, conveyor, controls));
  }

}
