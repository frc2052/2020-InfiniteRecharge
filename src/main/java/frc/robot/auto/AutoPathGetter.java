/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.Constants;
import java.util.List;

public class AutoPathGetter {
    TrajectoryConfig config = new TrajectoryConfig(Constants.Autonomous.maxVelocity, Constants.Autonomous.maxAcceleration);
    //TODO: potentially fix this later
    public Trajectory getTrajectory() {
        switch(AutoModeSelector.getSelectedAuto()) {
            case CS: 
                Trajectory CS = TrajectoryGenerator.generateTrajectory(
                    new Pose2d(3.048, 0, new Rotation2d(0)),
                    List.of(
                        new Translation2d(3.048, 0)
                    ),
                    new Pose2d(3.81, 0, new Rotation2d(0)), 
                    config
                );
                return CS;
            case CSG3: //B, C, D, E
                Trajectory CSG3 = TrajectoryGenerator.generateTrajectory(
                    new Pose2d(3.048, 0, new Rotation2d(0)),
                    List.of(
                        new Translation2d(5.2324, -2.0066), 
                        new Translation2d(6.1468, -2.0066)
                    ),
                    new Pose2d(5.3086,  -1.397, new Rotation2d(0)),
                    config
                );
                return CSG3;
            case LSG3: // i, B, C, D
                Trajectory LSG3 = TrajectoryGenerator.generateTrajectory(
                    new Pose2d(3.048, -1.7018, new Rotation2d(0)),
                    List.of(
                        new Translation2d(5.2324, -2.0066), 
                        new Translation2d(6.1468, -2.0066)
                    ),
                    new Pose2d(5.3086,  -1.397, new Rotation2d(0)),
                    config
                );
                return LSG3;
            case LST2: //i, H, F, G
                Trajectory LST2 = TrajectoryGenerator.generateTrajectory(
                    new Pose2d(3.048, -1.7018, new Rotation2d(0)),
                    List.of(
                        new Translation2d(6.35, -5.334), 
                        new Translation2d(6.5024, -5.08)
                    ),
                    new Pose2d(6.35,  -4.8514, new Rotation2d(0)),
                    config
                );
                return LST2;
            case LSG5: //i, B, C, D, M, N, O
                Trajectory LSG5 = TrajectoryGenerator.generateTrajectory(
                    new Pose2d(3.048, -1.7018, new Rotation2d(0)),
                    List.of(
                        new Translation2d(5.2324, -2.0066), 
                        new Translation2d(6.1468, -2.0066),
                        new Translation2d(5.3086,  -1.397),
                        new Translation2d(5.7658, -1.0922), 
                        new Translation2d(6.223, -0.7874)
                    ),
                    new Pose2d(6.5278, -0.508, new Rotation2d(0)),
                    config
                );
                return LSG5;  
            case RST3: 
                Trajectory RST3 = TrajectoryGenerator.generateTrajectory(
                    new Pose2d(3.048, -1.7272, new Rotation2d(0)),
                    List.of(
                        new Translation2d(6.1722, -1.7272) 
                    ),
                    new Pose2d(8.001, -1.7272, new Rotation2d(0)),
                    config
                );
                return RST3; 
            default:
                Trajectory current = TrajectoryGenerator.generateTrajectory(
                    new Pose2d(3.048, 0, new Rotation2d(0)),
                    List.of(
                        new Translation2d(3.048, 0)
                    ),
                    new Pose2d(3.81, 0, new Rotation2d(0)), 
                    config
                );
                return current;
            }
        }
        
    }

