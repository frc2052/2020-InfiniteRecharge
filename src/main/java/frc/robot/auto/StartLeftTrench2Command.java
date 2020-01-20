/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.*;
import frc.robot.commands.DrivePathCommand.DrivePathEnum;
import frc.robot.subsystems.*;

public class StartLeftTrench2Command extends SequentialCommandGroup {
  /**
   * Creates a new StartLeftTrench2.
   */
  public StartLeftTrench2Command(DriveTrainSubystem driveTrain, ShooterSubsystem shooter, IntakeSubsystem intake, VisionSubsystem visionTracking) {
      this.addCommands(new ShootAllCommand(shooter, visionTracking));
      OuterIntakeInCommand intakeCmd = new OuterIntakeInCommand(intake);
      DrivePathCommand path1 = new DrivePathCommand(driveTrain, DrivePathEnum.StartLeftTrench2);
      ArmDownCommand armDownCmd = new ArmDownCommand(intake);
      ParallelCommandGroup par1 = new ParallelCommandGroup(intakeCmd, path1, armDownCmd);
      this.addCommands(par1);
      this.addCommands(new OuterIntakeStopCommand(intake));
      this.addCommands(new DrivePathCommand(driveTrain,DrivePathEnum.LeftTrenchToMiddle));
      this.addCommands(new ShootAllCommand(shooter, visionTracking));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
