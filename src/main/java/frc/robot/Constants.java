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

    public static final class Solenoids {
        public static final int kShifterSolenoidID = 4;
        public static final int kUpIntakeSolenoidID = 0;
        public static final int kDownIntakeSolenoidID = 1;
        public static final int kElevatorLockSolenoidID = 2;
        public static final int kElevatorUnLockSolenoidID = 3;
    }

    public static final class Motors {
        public static final int kDriveRightMasterId = 1;
        public static final int kDriveRightFollowerId = 2;
        public static final int kDriveLeftMasterId = 3;
        public static final int kDriveLeftFollowerId = 4;
        public static final int kOuterIntakeMotorID = 7;
        public static final int kClimberMotorID = 8;
        public static final int kShooterMasterMotorID = 9;
        public static final int kShooterFollowerMotorID = 10;
        public static final int kHoodMotorID = 11;
        public static final int kTurretMotorID = 12;
        public static final int kConveyorMotorBottemLeftID = 13;   
        public static final int kConveyorMotorBottemRightID = 14;   
        public static final int kLifterMotorID = 15;   
        public static final int kActiveBalanceMotorID = 16;
    }


    public static final class DriveTrain{
        public static final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(.7112);

       
        // See: http://docs.wpilib.org/en/latest/docs/software/wpilib-tools/robot-characterization/characterization-routine.html
        public static final double ksVolts = 0.53;
        public static final double kvVoltSecondsPerMeter = 4.89;
        public static final double kaVoltSecondsSquaredPerMeter = 0.559;
        
        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

        public static final int kVelocityControlSlot = 0;
        public static final int kCANBusConfigTimeoutMS = 10;
        public static final int kTicksPerRot = 1024;
        public static final double kEncoderGearRatio = (1.0/3)*(20.0/64);
        public static final double kDriveWheelCircumferenceInches = 6.0 * Math.PI;

        public static final int kPDriveVel = 8;

        public static final double kTurnInPlaceSpeed = .75;
    }

    public static final class Intake{
        public static final double kIntakeSpeed = 1;
    }

    public static final class Elevator{
       public static final double kPercentOutputUp = .2;
       public static final double kPercentOutputDown = -.2;
       public static final int kElevatorTicksPerRot = 4096;
       public static final int kElevatorInchesPerRotation = (int) (1.5 * Math.PI);
       public static final int kElevatorHeight = 63;
       public static final int kElevatorMinHeight = 0;
       public static final int kElevatorMaxHeight = 25000;
    }

    public static final class WheelOfFortune{
        public static final int kControlPanelID = 20;
        //TODO: find the correct id
        public static final double kMotorPower = .10;

    }

    public static final class Shooter{
        public static final int kCloseAnglePosition = 5 ;
        public static final int kMiddleAnglePosition = 10;
        public static final int kFarAnglePosition = 15;
        public static final int kAngleTicsPerRotations = 4096;
        public static final int kMaxAngleHeight = 20;
        public static final int kMinAngleHeight = 0;
        public static final double kEmergencyDownPower = -.25;
        public static final double kEmergencyUpPower = .25;
        public static final double kInchesPerRotation = 5;
        public static final double kShooterSpeedRPS = 5;

        public static final double kShooterF = (.9 * 1023) / 29900; //29900 is fastest we can run shooter
        public static final int kShooterTargetVelocity = 30000;
    }
  
    public static final class Turret{
        //public static final int kTurretMaxRight = 100;
        //public static final int kTurretMinLeft = -100;
        public static final double kTurnLeftSpeed = -0.1;
        public static final double kTurnRightSpeed = 0.1;

        public static final int kTurretMinEncoderPos = -4000;
        public static final int kTurretMaxEncoderPos = 16000;

        public static final double kTicksPerDegree = 4096 / 90;
        public static final double kMaxAngle = 90;
        public static final double kMinAngle = -90;

        public static final double kMinTurretSpeed = .1;
        public static final double kMaxTurretSpeed = .75;
    }
  
    public static class Autonomous {
        public static final double maxVelocity = 7 * 12;
        public static final double maxAcceleration = 120;

        public static final int kLeftInitTargetTicks = 20000;
        public static final int kRightInitTargetTicks = -20000;
    }

    public static final class Vision {
        public static final double kCameraViewHorDegrees = 54.0;
        public static final double kCameraViewVertDegrees = 41.0;
        public static final double kCameraViewHorPixels = 320.0;
        public static final double kCameraViewVertPixels = 240.0;
        public static final double kCameraMaxViewHorDegrees = 180.0;

        public static final double kCameraMountingAngleY = 28; // mounting angle of the camera     DEGREESEE
        public static final double kCameraMountingHeight = 37.5; // height of camera off the ground  INCHES
        public static final double kTargetHeight = 89.5;// height of target off the ground  INCHES  top is 98 bottom is 81
    }

    public static final class ConveyorSubsystem {      
        public static final boolean klifterdirection = true;
        public static final boolean kConveyorDirection = true;
        public static final double kConveyorSpeed = 0.75;
        //Todo: find the actual port
        public static final int kMiddleBallSensorID = 0;
        public static final int kFrontBallSensorID = 1;
        public static final int kTopBallSensorID = 2;
    }

    public static final class Hood {
        public static final int kTicksPerRotation = 4096;
        public static final double kHoodDownSpeed = -.25;
        public static final double kHoodUpSpeed = .25;
        //TODO manualy find max and min endcoder values 
        public static final double kMinHoodTicks = 0;
        public static final int kMaxHoodTicks = 1250;

        public static final double kMaxHoodSpeed = .35;
        public static final double kMinHoodSpeed = .18;
    }
    
    public static final class SmartDashboardStrings {
        public static final String kHoodOverrideString = "Hood Override?";
        public static final String kTurretOverrideString = "Turret Override?";
        public static final String kShooterOverrideString = "Shooter Override?";
        public static final String kAutoBumpString = "Auto Bump?";
        public static final String kTimeToShoot = "Auto time to shoot?";
    }
    public static final class PixyCamDriveConstants {

        public static final int minMidPixelRange = 125;
        public static final int maxMidPixelRange = 175;
        public static final int minClosePixelRange = 75;
        public static final int maxClosePixelRange = 225;
        public static final double turnSpeed = .5;
        public static final double pixyCamDriveForwardSpeed = -.6;

    }
}
