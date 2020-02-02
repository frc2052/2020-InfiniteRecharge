/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import java.util.List;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.Constants;
import frc.robot.auto.AutoModeSelector;


public class DrivePathCommand extends CommandBase {
  private final DriveTrainSubsystem m_driveTrainSubsystem;
  private final DrivePathEnum m_choosenPath;

  private double centerStartX = 3.048;
  private double centerStartY = 0;

  public DrivePathCommand(DriveTrainSubsystem driveTrain, DrivePathEnum pathEnum) {
    addRequirements(driveTrain);
    m_driveTrainSubsystem = driveTrain;
    m_choosenPath = pathEnum;

    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  public double getStartingX() {
    switch(AutoModeSelector.getPosOnLine()) {
      case MIDDLE:
        return centerStartX;
      case FORWARD:
        return centerStartX - AutoModeSelector.getDistanceOffLine();
      case BACK:
        return centerStartX + AutoModeSelector.getDistanceOffLine();
      default:
        return 0;
    }
  }
  

  public double getStartingY() {
    double centerDistanceToLeftWall = 7.8;
    double centerDistanceToRightWall = 3.5; //TODO: figure out correct values

    switch(AutoModeSelector.getDirectionMeasured()) {
      case LEFT:
        return centerDistanceToLeftWall + AutoModeSelector.getDistanceFromWall();
      case RIGHT:
        return centerDistanceToRightWall - AutoModeSelector.getDistanceFromWall();
      default:
        return centerStartY;
    }
  }

  public Trajectory getTrajectory(DrivePathEnum selectedPath) {
      var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
              new SimpleMotorFeedforward(
                Constants.DriveTrain.ksVolts, 
                Constants.DriveTrain.kvVoltSecondsPerMeter,
                Constants.DriveTrain.kaVoltSecondsSquaredPerMeter
              ),
              Constants.DriveTrain.kinematics, 10);

      TrajectoryConfig config = new TrajectoryConfig(
        Constants.Autonomous.maxVelocity,
        Constants.Autonomous.maxAcceleration
      ).setKinematics(Constants.DriveTrain.kinematics)
        .addConstraint(autoVoltageConstraint);

      switch(selectedPath)  {
        case StartCenterDriveBackPark:
            Trajectory CS = TrajectoryGenerator.generateTrajectory(new Pose2d(3.048, 0, new Rotation2d(0)),
              List.of(new Translation2d(3.048, 0)), new Pose2d(3.81, 0, new Rotation2d(0)), config); 
            return CS;
        case StartCenterGenerator3:
            Trajectory CSG3 = TrajectoryGenerator.generateTrajectory(new Pose2d(3.048, 0, new Rotation2d(0)),
              List.of(new Translation2d(5.2324, -2.0066), new Translation2d(6.1468, -2.0066)),
            new Pose2d(5.3086, -1.397, new Rotation2d(0)), config);
          return CSG3;
        case StartLeftGenerator3:
            Trajectory LSG3 = TrajectoryGenerator.generateTrajectory(new Pose2d(3.048, -1.7018, new Rotation2d(0)),
              List.of(new Translation2d(5.2324, -2.0066), new Translation2d(6.1468, -2.0066)),
                new Pose2d(5.3086, -1.397, new Rotation2d(0)), config);
            return LSG3;
        case StartLeftGenerator5:
            Trajectory LSG5 = TrajectoryGenerator.generateTrajectory(new Pose2d(3.048, -1.7018, new Rotation2d(0)),
              List.of(new Translation2d(5.2324, -2.0066), new Translation2d(6.1468, -2.0066),
                new Translation2d(5.3086, -1.397), new Translation2d(5.7658, -1.0922),
                new Translation2d(6.223, -0.7874)),
              new Pose2d(6.5278, -0.508, new Rotation2d(0)), config);
            return LSG5;
        case StartLeftTrench2:
            Trajectory LST2 = TrajectoryGenerator.generateTrajectory(new Pose2d(3.048, -1.7018, new Rotation2d(0)),
              List.of(new Translation2d(6.35, -5.334), new Translation2d(6.5024, -5.08)),
            new Pose2d(6.35, -4.8514, new Rotation2d(0)), config);
          return LST2;
        case StartRightTrench3Ball:
            Trajectory RST3 = TrajectoryGenerator.generateTrajectory(new Pose2d(3.048, -1.7272, new Rotation2d(0)),
              List.of(new Translation2d(6.1722, -1.7272)), new Pose2d(8.001, -1.7272, new Rotation2d(0)), config);
          return RST3;
        case LeftTrenchToMiddle: //TODO: fix these values
            Trajectory LTM = TrajectoryGenerator.generateTrajectory(new Pose2d(3.048, -1.7272, new Rotation2d(0)),
              List.of(new Translation2d(6.1722, -1.7272)), new Pose2d(8.001, -1.7272, new Rotation2d(0)), config);
          return LTM;
          case CenterGenerator5:
            Trajectory CG5 = TrajectoryGenerator.generateTrajectory(new Pose2d(3.048, -1.7272, new Rotation2d(0)),
            List.of(new Translation2d(6.1722, -1.7272)), new Pose2d(8.001, -1.7272, new Rotation2d(0)), config);
            return CG5;
        default:
            Trajectory DontMove = TrajectoryGenerator.generateTrajectory(new Pose2d(3.048, 0, new Rotation2d(0)),
              List.of(new Translation2d(3.048, 0)), new Pose2d(3.81, 0, new Rotation2d(0)), config);
          return DontMove;
      }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RamseteCommand ramseteCommand = new RamseteCommand(
      getTrajectory(m_choosenPath),
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
    StartCenterGenerator3,
    StartRightTrench3Ball,
    StartLeftGenerator3,
    StartLeftTrench2,
    StartLeftGenerator5,
    LeftTrenchToMiddle,
    CenterGenerator5
  }

}