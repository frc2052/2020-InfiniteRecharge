/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;

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

import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Constants;

public class TrajectoryFactory {
    private DriveTrainSubsystem m_driveTrainSubsystem;

    public RamseteCommand getRamseteCommand(DriveTrainSubsystem driveTrain, DrivePathEnum selectedPath) {
        Trajectory path = getTrajectory(selectedPath);
        m_driveTrainSubsystem = driveTrain;

        RamseteCommand ramseteCommand = new RamseteCommand(
            path,
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

        return ramseteCommand;
    }

    public Trajectory getTrajectory(DrivePathEnum selectedPath) {
        Pose2d startPos = new Pose2d(Units.inchesToMeters(138), Units.inchesToMeters(AutoModeSelector.getPosOnLineInches()), new Rotation2d(0));

        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(
                  Constants.DriveTrain.ksVolts, 
                  Constants.DriveTrain.kvVoltSecondsPerMeter,
                  Constants.DriveTrain.kaVoltSecondsSquaredPerMeter
                ),
                Constants.DriveTrain.kinematics, 10);
  
        TrajectoryConfig forwardConfig = new TrajectoryConfig(
            Constants.Autonomous.maxVelocity,
            Constants.Autonomous.maxAcceleration)
            .setKinematics(Constants.DriveTrain.kinematics)
            .addConstraint(autoVoltageConstraint);

        TrajectoryConfig backwardsConfig = new TrajectoryConfig(
            Constants.Autonomous.maxVelocity,
            Constants.Autonomous.maxAcceleration)
            .setKinematics(Constants.DriveTrain.kinematics)
            .addConstraint(autoVoltageConstraint)
            .setReversed(true);
  
        switch(selectedPath)  {
          case StartCenterDriveBackPark:
              return TrajectoryGenerator.generateTrajectory(
                    startPos, //start, B
                        new ArrayList<Translation2d>(), //can't use a list if we have no points
                    new Pose2d(Units.inchesToMeters(168), 0, new Rotation2d(0)), forwardConfig);  //end, 30 inches away from start line
          case StartCenterGenerator3:
                return TrajectoryGenerator.generateTrajectory(
                    startPos, 
                        List.of(
                            new Translation2d(Units.inchesToMeters(206), Units.inchesToMeters(-79)), // C
                            new Translation2d(Units.inchesToMeters(242), Units.inchesToMeters(-79))), // D
                    new Pose2d(Units.inchesToMeters(209), Units.inchesToMeters(-55), new Rotation2d(0)), forwardConfig); // E
          case StartLeftGenerator3:
                return TrajectoryGenerator.generateTrajectory(
                    new Pose2d(3.048, -1.7018, new Rotation2d(0)),
                        List.of(
                            new Translation2d(5.2324, -2.0066), 
                            new Translation2d(6.1468, -2.0066)),
                    new Pose2d(5.3086, -1.397, new Rotation2d(0)), forwardConfig);
          case StartLeftGenerator5:
                return TrajectoryGenerator.generateTrajectory(
                    new Pose2d(3.048, -1.7018, new Rotation2d(0)),
                        List.of(
                            new Translation2d(5.2324, -2.0066), 
                            new Translation2d(6.1468, -2.0066),
                            new Translation2d(5.3086, -1.397),
                            new Translation2d(5.7658, -1.0922),
                            new Translation2d(6.223, -0.7874)),
                    new Pose2d(6.5278, -0.508, new Rotation2d(0)), forwardConfig);
          case StartLeftTrench2:
                return TrajectoryGenerator.generateTrajectory(
                    new Pose2d(3.048, -1.7018, new Rotation2d(0)),
                        List.of(
                            new Translation2d(6.35, -5.334),
                            new Translation2d(6.5024, -5.08)),
                    new Pose2d(6.35, -4.8514, new Rotation2d(0)), forwardConfig);
          case StartRightTrench3Ball:
                return TrajectoryGenerator.generateTrajectory(
                    new Pose2d(3.048, -1.7272, new Rotation2d(0)),
                        List.of(
                            new Translation2d(6.1722, -1.7272)), 
                    new Pose2d(8.001, -1.7272, new Rotation2d(0)), forwardConfig);
          case LeftTrenchToMiddle: //TODO: fix these values
                return TrajectoryGenerator.generateTrajectory(
                    new Pose2d(3.048, -1.7272, new Rotation2d(0)),
                        List.of(
                            new Translation2d(6.1722, -1.7272)),
                    new Pose2d(8.001, -1.7272, new Rotation2d(0)), forwardConfig);
            case CenterGenerator5:
                return TrajectoryGenerator.generateTrajectory(
                    new Pose2d(3.048, -1.7272, new Rotation2d(0)),
                        List.of(
                            new Translation2d(6.1722, -1.7272)), 
                    new Pose2d(8.001, -1.7272, new Rotation2d(0)), forwardConfig);
            case AutoFeed:
                return TrajectoryGenerator.generateTrajectory(
                    new Pose2d(3.048, -1.7272, new Rotation2d(0)),
                        List.of(
                            new Translation2d(6.1722, -1.7272)), 
                    new Pose2d(8.001, -1.7272, new Rotation2d(0)), forwardConfig);
          default:
                return TrajectoryGenerator.generateTrajectory(
                    new Pose2d(3.048, 0, new Rotation2d(0)),
                        List.of(
                            new Translation2d(3.048, 0)), 
                    new Pose2d(3.81, 0, new Rotation2d(0)), forwardConfig);
        }
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
      CenterGenerator5,
      AutoFeed
    }

}
