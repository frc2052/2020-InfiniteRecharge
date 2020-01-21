/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.auto.AutoPathGetter;
import frc.robot.subsystems.DriveTrainSubsystem;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.controller.PIDController;
import frc.robot.Constants;

public class DrivePathCommand extends CommandBase {
 
  private final DriveTrainSubsystem m_driveTrainSubsystem;

  public DrivePathCommand(DriveTrainSubsystem driveTrain) {
    m_driveTrainSubsystem = driveTrain;

    addRequirements(driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RamseteCommand ramseteCommand = new RamseteCommand(
        AutoPathGetter.getSelectedTrajectory(),
        m_driveTrainSubsystem::getPose,
        new RamseteController(Constants.DriveTrain.kRamseteB, Constants.DriveTrain.kRamseteZeta),
        new SimpleMotorFeedforward(Constants.DriveTrain.ksVolts,
                                   Constants.DriveTrain.kvVoltSecondsPerMeter,
                                   Constants.DriveTrain.kaVoltSecondsSquaredPerMeter),
        Constants.DriveTrain.kinematics,
        m_driveTrainSubsystem::getWheelSpeeds,
        new PIDController(Constants.DriveTrain.kPDriveVel, 0, 0),
        new PIDController(Constants.DriveTrain.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_driveTrainSubsystem::tankDriveVolts,
        m_driveTrainSubsystem
    );

    ramseteCommand.andThen(() -> m_driveTrainSubsystem.tankDriveVolts(0, 0));
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

  public enum DrivePathEnum
  {
    StartCenterDriveBackPark,
    StartRightTrench3Ball,
    StartLeftGenerator3,
    StartLeftTrench2,
    StartLeftGenerator5,
    LeftTrenchToMiddle
  }

}
