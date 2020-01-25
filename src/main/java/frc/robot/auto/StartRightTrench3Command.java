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
import frc.robot.commands.*;
import frc.robot.commands.DrivePathCommand.DrivePathEnum;
import frc.robot.subsystems.*;

public class StartRightTrench3Command extends SequentialCommandGroup {
  /**
   * Creates a new StartRightTrench3Command.
   */

  public StartRightTrench3Command(DriveTrainSubsystem driveTrain, ShooterSubsystem shooter, IntakeSubsystem intake, VisionSubsystem vision, HoodSubsystem hood, Double  delayTime) {
	this.addCommands(new WaitCommand(delayTime));
    this.addCommands(new ShootAllCommand(shooter, vision, hood));    
	OuterIntakeInCommand intakeCmd = new OuterIntakeInCommand(intake);
    DrivePathCommand path1 = new DrivePathCommand(driveTrain, DrivePathEnum.StartRightTrench3Ball);
    ArmDownCommand armDownCmd = new ArmDownCommand(intake);
    ParallelCommandGroup par1 = new ParallelCommandGroup(intakeCmd, path1, armDownCmd);
    this.addCommands(par1);
    this.addCommands(new OuterIntakeStopCommand(intake));
    this.addCommands(new ShootAllCommand(shooter, vision, hood));
  }

}
