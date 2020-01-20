/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveTrain{
        public static final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(.7112);

        // TODO - Find the real values. DO NOT RUN THE ROBOT WITH THESE
        // See: http://docs.wpilib.org/en/latest/docs/software/wpilib-tools/robot-characterization/characterization-routine.html
        public static final double ksVolts = 0.22;
        public static final double kvVoltSecondsPerMeter = 1.98;
        public static final double kaVoltSecondsSquaredPerMeter = 0.2;
        
        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

        public static final int kDriveRightMasterId = 1;
        public static final int kDriveRightSlaveId = 2;
        public static final int kDriveRightSlave2Id = 3;
        public static final int kDriveLeftMasterId = 4;
        public static final int kDriveLeftSlaveId = 5;
        public static final int kDriveLeftSlave2Id = 6;


        public static final int kVelocityControlSlot = 0;
        public static final int kCANBusConfigTimeoutMS = 10;
        public static final int kTicksPerRot = 1024;
        public static final double kEncoderGearRatio = (1.0/3)*(20.0/64);
        public static final double kDriveWheelCircumferenceInches = 6.0 * Math.PI;

        public static final int kShiftInSolenoidID = 0;
        public static final int kShiftOutSolenoidID = 1;

        public static final double kTurnInPlaceSpeed = .75;
    }
    public static final class Intake{
        public static final int kUpIntakeSolenoidID = 2;
        public static final int kDownIntakeSolenoidID = 3;
        public static final int kOuterIntakeMotorID = 7;
    }
    public static class Autonomous {
        public static final double maxVelocity = 7 * 12;
        public static final double maxAcceleration = 120;
    }
}
