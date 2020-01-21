/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.Constants;
import java.util.List;

public class AutoPathGetter {
    static Trajectory chosenPath;

    public void setTrajectory() {
        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(Constants.DriveTrain.ksVolts, Constants.DriveTrain.kvVoltSecondsPerMeter,
                        Constants.DriveTrain.kaVoltSecondsSquaredPerMeter),
                Constants.DriveTrain.kinematics, 10);

        TrajectoryConfig config = new TrajectoryConfig(Constants.Autonomous.maxVelocity,
                Constants.Autonomous.maxAcceleration).setKinematics(Constants.DriveTrain.kinematics)
                        .addConstraint(autoVoltageConstraint);

        switch (AutoModeSelector.getSelectedAuto()) {
            case CS:
                chosenPath = TrajectoryGenerator.generateTrajectory(new Pose2d(3.048, 0, new Rotation2d(0)),
                        List.of(new Translation2d(3.048, 0)), new Pose2d(3.81, 0, new Rotation2d(0)), config);
                break;
            case CSG3: // B, C, D, E
                chosenPath = TrajectoryGenerator.generateTrajectory(new Pose2d(3.048, 0, new Rotation2d(0)),
                        List.of(new Translation2d(5.2324, -2.0066), new Translation2d(6.1468, -2.0066)),
                        new Pose2d(5.3086, -1.397, new Rotation2d(0)), config);
                break;
            case LSG3: // i, B, C, D
                chosenPath = TrajectoryGenerator.generateTrajectory(new Pose2d(3.048, -1.7018, new Rotation2d(0)),
                        List.of(new Translation2d(5.2324, -2.0066), new Translation2d(6.1468, -2.0066)),
                        new Pose2d(5.3086, -1.397, new Rotation2d(0)), config);
                break;
            case LST2: // i, H, F, G
                chosenPath = TrajectoryGenerator.generateTrajectory(new Pose2d(3.048, -1.7018, new Rotation2d(0)),
                        List.of(new Translation2d(6.35, -5.334), new Translation2d(6.5024, -5.08)),
                        new Pose2d(6.35, -4.8514, new Rotation2d(0)), config);
                break;
            case LSG5: // i, B, C, D, M, N, O
                chosenPath = TrajectoryGenerator.generateTrajectory(new Pose2d(3.048, -1.7018, new Rotation2d(0)),
                        List.of(new Translation2d(5.2324, -2.0066), new Translation2d(6.1468, -2.0066),
                                new Translation2d(5.3086, -1.397), new Translation2d(5.7658, -1.0922),
                                new Translation2d(6.223, -0.7874)),
                        new Pose2d(6.5278, -0.508, new Rotation2d(0)), config);
                break;
            case RST3:
                chosenPath = TrajectoryGenerator.generateTrajectory(new Pose2d(3.048, -1.7272, new Rotation2d(0)),
                        List.of(new Translation2d(6.1722, -1.7272)), new Pose2d(8.001, -1.7272, new Rotation2d(0)), config);
                break;
            default:
                chosenPath = TrajectoryGenerator.generateTrajectory(new Pose2d(3.048, 0, new Rotation2d(0)),
                        List.of(new Translation2d(3.048, 0)), new Pose2d(3.81, 0, new Rotation2d(0)), config);
        }
    }

    public static Trajectory getSelectedTrajectory() {
        return chosenPath;
    }
}
