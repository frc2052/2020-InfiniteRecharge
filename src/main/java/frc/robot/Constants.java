/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

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

        public static final int kPDriveVel = 8;

        public static final int kShiftInSolenoidID = 0;
        public static final int kShiftOutSolenoidID = 1;

        public static final double kTurnInPlaceSpeed = .75;
    }

    public static final class Intake{
        public static final int kUpIntakeSolenoidID = 2;
        public static final int kDownIntakeSolenoidID = 3;
        public static final int kOuterIntakeMotorID = 7;
    }
    public static final class Shooter{
        public static final int kShooterMotorID = 8;
        public static final int kAngleMotorID = 9;
        public static final int kCloseAnglePosition = 5 ;
        public static final int kMiddleAnglePosition = 10;
        public static final int kFarAnglePosition = 15;
        public static final int kAngleTicsPerRotations = 4096;
        public static final int kMaxAngleHeight = 20;
        public static final int kMinAngleHeight = 0;
        public static final double kEmergencyDownPower = -.25;
        public static final double kEmergencyHoldPower = .1;
        public static final double kEmergencyUpPower = .25;
        public static final double kInchesPerRotation = 5;
        public static final double kShooterSpeedRPS = 5;
    }
    public static final class Turret{
        public static final int kTurretMotorID = 10;
        public static final int kTurretMaxRight = 100;
        public static final int kTurretMinLeft = -100;
        public static final double kTurnLeftSpeed = -0.1;
        public static final double kTurnRightSpeed = 0.1;
    }
    public static class Autonomous {
        public static final double maxVelocity = 7 * 12;
        public static final double maxAcceleration = 120;
    }

    public static final class Vision {
        public static final double limelightHeight = 20.0;
        public static final int targetHeight = 249; //cm
        public static final int mountingAngle = 0;
    }

    public static final class ConveyorSubsystem {      
        public static final int kConveyorMotorBottemLeftID = 0;   
        public static final int kConveyorMotorBottemRightID = 0;   
        public static final int kLifterFrontID = 0;   
        public static final int kLifterBackID = 0;
        public static final boolean klifterdirection = true;
        public static final boolean kconveyordirection = true;
    }
}
