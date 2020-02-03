/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.*;
import frc.robot.commands.DrivePathCommand.DrivePathEnum;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class StartLeftShoot5Command extends SequentialCommandGroup {
  /**
   * Creates a new AutoLeftShoot5.
   */
  public StartLeftShoot5Command(DriveTrainSubsystem driveTrain, ShooterSubsystem shooter, IntakeSubsystem intake, VisionSubsystem vision, HoodSubsystem hood, TurretSubsystem turret, ConveyorSubsystem conveyor, Double delayTime) {
    this.addCommands(new WaitCommand(delayTime));
    this.addCommands(new MegaShooterCommand(shooter, vision, hood, turret, conveyor));
    ArmDownCommand intakeCmd = new ArmDownCommand(intake);
    DrivePathCommand path1 = new DrivePathCommand(driveTrain, DrivePathEnum.CenterGenerator5);
    ArmDownCommand armDownCmd = new ArmDownCommand(intake);
    ParallelCommandGroup par1 = new ParallelCommandGroup(intakeCmd, path1, armDownCmd);
    this.addCommands(par1);
    this.addCommands(new OuterIntakeStopCommand(intake));
    this.addCommands(new MegaShooterCommand(shooter, vision, hood, turret, conveyor));
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
