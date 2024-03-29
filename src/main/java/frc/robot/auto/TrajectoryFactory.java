/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;

import frc.robot.subsystems.DriveTrainSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
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
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;

import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Constants;

public class TrajectoryFactory {
    private DriveTrainSubsystem m_driveTrainSubsystem;

    public Command getRamseteCommand(DriveTrainSubsystem driveTrain, DrivePathEnum selectedPath) {
        Trajectory path = getTrajectory(selectedPath);
        m_driveTrainSubsystem = driveTrain;
        if (driveTrain == null)
        {
            System.err.println("DRIVE TRAIN NULL^^^^^^^^^^^^^^^^^^^^^^^^^^");
        }

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
        //AutoModeSelector.getPosOnLineInches()
        Pose2d startPos = new Pose2d(Units.inchesToMeters(138), Units.inchesToMeters(AutoModeSelector.getPosOnLineInches()), new Rotation2d(0));
        Pose2d centerStartPos = new Pose2d(Units.inchesToMeters(138), Units.inchesToMeters(0), new Rotation2d(0));
        Pose2d leftStartPos = new Pose2d(Units.inchesToMeters(138), Units.inchesToMeters(-220), new Rotation2d(0));
        Pose2d rightStartPos = new Pose2d(Units.inchesToMeters(138), Units.inchesToMeters(68), new Rotation2d(0));

        Pose2d generatorPrep = new Pose2d(Units.inchesToMeters(242), Units.inchesToMeters(79), new Rotation2d(Math.toRadians(0)));
        Pose2d generator3End = new Pose2d(Units.inchesToMeters(209), Units.inchesToMeters(55), new Rotation2d(0));
        Pose2d trenchBall3 = new Pose2d(Units.inchesToMeters(281), Units.inchesToMeters(68), new Rotation2d(0));

        Pose2d generator2Ball = new Pose2d(Units.inchesToMeters(227), Units.inchesToMeters(-6), new Rotation2d(Math.toRadians(-60)));
        
        Translation2d generator2BallPrep = new Translation2d(Units.inchesToMeters(198), Units.inchesToMeters(30));
        Translation2d trenchBall1 = new Translation2d(Units.inchesToMeters(200), Units.inchesToMeters(68));
        Translation2d trenchBall2 = new Translation2d(Units.inchesToMeters(240), Units.inchesToMeters(68));

        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(
                  Constants.DriveTrain.ksVolts, 
                  Constants.DriveTrain.kvVoltSecondsPerMeter,
                  Constants.DriveTrain.kaVoltSecondsSquaredPerMeter
                ),
                Constants.DriveTrain.kinematics, 10);

        TrajectoryConfig startingConfig = new TrajectoryConfig( //used if we only have one part of a path
            Constants.Autonomous.maxVelocity,
            Constants.Autonomous.maxAcceleration)
            .setKinematics(Constants.DriveTrain.kinematics)
            .addConstraint(autoVoltageConstraint);

        CentripetalAccelerationConstraint turnConstraint = new CentripetalAccelerationConstraint(.75);
  
        TrajectoryConfig forwardsConfig = new TrajectoryConfig( //used if there is multiple parts to a path
            5, //5 //1
            2) //2 //.25
            .setKinematics(Constants.DriveTrain.kinematics)
            .addConstraint(autoVoltageConstraint)
            .addConstraint(turnConstraint);
            //.setEndVelocity(.5);
        
        TrajectoryConfig slowConfig = new TrajectoryConfig(
            1,
            .25)
            .setKinematics(Constants.DriveTrain.kinematics)
            .addConstraint(autoVoltageConstraint)
            .addConstraint(turnConstraint);

        TrajectoryConfig backwardsConfig = new TrajectoryConfig(
            5,
            2)
            .setKinematics(Constants.DriveTrain.kinematics)
            .addConstraint(autoVoltageConstraint)
            .addConstraint(turnConstraint)
            .setReversed(true);

        TrajectoryConfig midPathConfig = new TrajectoryConfig(
            Constants.Autonomous.maxVelocity,
            Constants.Autonomous.maxAcceleration)
            .setKinematics(Constants.DriveTrain.kinematics)
            .addConstraint(autoVoltageConstraint)
            .setStartVelocity(.75);
  
        switch(selectedPath)  {
            case StartCenterDriveBackPark:
              return TrajectoryGenerator.generateTrajectory(
                    centerStartPos, //start, B
                        new ArrayList<Translation2d>(), //can't use a list if we have no points
                    new Pose2d(Units.inchesToMeters(158), 0, new Rotation2d(0)), startingConfig);  //end, 30 inches away from start line
            case StartToGenerator:
                return TrajectoryGenerator.generateTrajectory(
                    startPos, 
                        List.of(
                            new Translation2d(Units.inchesToMeters(206), Units.inchesToMeters(79))), 
                    generatorPrep, forwardsConfig);
            case CenterLineToGen2:              
                return TrajectoryGenerator.generateTrajectory(
                    centerStartPos,
                        List.of(
                            new Translation2d(Units.inchesToMeters(215), Units.inchesToMeters(0))),
                            //new Translation2d(Units.inchesToMeters(263), Units.inchesToMeters(-20))), 
                        new Pose2d(Units.inchesToMeters(235), Units.inchesToMeters(-20), new Rotation2d(Math.toRadians(-50))),slowConfig);
            case Gen2Backup:
                return TrajectoryGenerator.generateTrajectory(
                    new Pose2d(Units.inchesToMeters(235), Units.inchesToMeters(-20), new Rotation2d(Math.toRadians(-50))), 
                        new ArrayList<Translation2d>(), 
                    new Pose2d(Units.inchesToMeters(223), Units.inchesToMeters(-8), new Rotation2d(Math.toRadians(-50))), backwardsConfig);
            case GeneratorBallPath:
                return TrajectoryGenerator.generateTrajectory(
                    startPos,
                        new ArrayList<Translation2d>(), 
                    generator3End, backwardsConfig);
            case StartLeftGenerator3:
                return TrajectoryGenerator.generateTrajectory(
                    new Pose2d(3.048, -1.7018, new Rotation2d(0)),
                        List.of(
                            new Translation2d(5.2324, -2.0066), 
                            new Translation2d(6.1468, -2.0066)),
                    new Pose2d(5.3086, -1.397, new Rotation2d(0)), forwardsConfig);
            case StartLeftGenerator5:
                return TrajectoryGenerator.generateTrajectory(
                    new Pose2d(3.048, -1.7018, new Rotation2d(0)),
                        List.of(
                            new Translation2d(5.2324, -2.0066), 
                            new Translation2d(6.1468, -2.0066),
                            new Translation2d(5.3086, -1.397),
                            new Translation2d(5.7658, -1.0922),
                            new Translation2d(6.223, -0.7874)),
                    new Pose2d(6.5278, -0.508, new Rotation2d(0)), forwardsConfig);
            case StartLeftTrench2:
                return TrajectoryGenerator.generateTrajectory(
                    leftStartPos,
                        new ArrayList<Translation2d>(),
                    new Pose2d(Units.inchesToMeters(214), Units.inchesToMeters(-220), new Rotation2d(0)), forwardsConfig);
            case Trench2ToShoot:
                return TrajectoryGenerator.generateTrajectory(
                    new Pose2d(Units.inchesToMeters(214),  Units.inchesToMeters(-220), new Rotation2d(0)), 
                        new ArrayList<Translation2d>(),
                        // List.of(
                        //     new Translation2d(Units.inchesToMeters(175), Units.inchesToMeters(-112))), 
                    new Pose2d(Units.inchesToMeters(150), Units.inchesToMeters(-75), new Rotation2d(Math.toRadians(-20))), backwardsConfig);
            case LeftTrenchBackup:
                return TrajectoryGenerator.generateTrajectory(
                    new Pose2d(Units.inchesToMeters(150), Units.inchesToMeters(-75), new Rotation2d(Math.toRadians(-20))), 
                        new ArrayList<Translation2d>(),
                    new Pose2d(Units.inchesToMeters(174), Units.inchesToMeters(-75), new Rotation2d(Math.toRadians(0))), forwardsConfig);
            case RightLineToTrenchMiddle:
                return TrajectoryGenerator.generateTrajectory(
                    rightStartPos, 
                        new ArrayList<Translation2d>(), 
                    new Pose2d(trenchBall2, new Rotation2d(0)), forwardsConfig);
            case LineToTrenchBack:
                return TrajectoryGenerator.generateTrajectory(
                    rightStartPos, 
                        new ArrayList<Translation2d>(),  
                    trenchBall3, forwardsConfig);
            case TrenchMiddleToBack:
                return TrajectoryGenerator.generateTrajectory(
                    new Pose2d(trenchBall2, new Rotation2d(0)), 
                        new ArrayList<Translation2d>(), 
                    trenchBall3, forwardsConfig);
            case TrenchFrontToBack:
                return TrajectoryGenerator.generateTrajectory(
                    new Pose2d(trenchBall1, new Rotation2d(0)),
                        new ArrayList<Translation2d>(), 
                    trenchBall3, forwardsConfig);
            case TrenchBackToLine:
                return TrajectoryGenerator.generateTrajectory(
                    trenchBall3,
                        new ArrayList<Translation2d>(), 
                    new Pose2d(Units.inchesToMeters(160), Units.inchesToMeters(68), new Rotation2d(0)), backwardsConfig);
          case LeftTrenchToMiddle: //TODO: fix these values
                return TrajectoryGenerator.generateTrajectory(
                    new Pose2d(3.048, -1.7272, new Rotation2d(0)),
                        List.of(
                            new Translation2d(6.1722, -1.7272)),
                    new Pose2d(8.001, -1.7272, new Rotation2d(0)), forwardsConfig);
            case CenterGenerator5:
                return TrajectoryGenerator.generateTrajectory(
                    new Pose2d(3.048, -1.7272, new Rotation2d(0)),
                        List.of(
                            new Translation2d(6.1722, -1.7272)), 
                    new Pose2d(8.001, -1.7272, new Rotation2d(0)), forwardsConfig);
            case AutoFeed:
                return TrajectoryGenerator.generateTrajectory(
                    new Pose2d(3.048, -1.7272, new Rotation2d(0)),
                        List.of(
                            new Translation2d(6.1722, -1.7272)), 
                    new Pose2d(8.001, -1.7272, new Rotation2d(0)), forwardsConfig);
          default:
                return TrajectoryGenerator.generateTrajectory(
                    new Pose2d(3.048, 0, new Rotation2d(0)),
                        List.of(
                            new Translation2d(3.048, 0)), 
                    new Pose2d(3.81, 0, new Rotation2d(0)), forwardsConfig);
        }
    }

    public enum DrivePathEnum
    {
      RightLineToTrenchMiddle,
      LeftTrenchBackup,
      Gen2Backup,
      LineToTrenchBack,
      //TrenchFrontToMiddle,
      TrenchMiddleToBack,
      CenterLineToGen2,
      StartCenterDriveBackPark,
      StartToGenerator,
      GeneratorBallPath,
      TrenchFrontToBack,
      TrenchBackToLine,
      StartLeftGenerator3,
      StartLeftTrench2,
      StartLeftGenerator5,
      LeftTrenchToMiddle,
      CenterGenerator5,
      AutoFeed,
      Trench2ToShoot
    }

}
