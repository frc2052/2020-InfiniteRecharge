/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class DriveTrainSubsystem extends SubsystemBase {

  private WPI_TalonSRX leftMaster;
  private VictorSPX leftFollower1; 
  private VictorSPX leftFollower2;
  
  private final WPI_TalonSRX rightMaster;
  private final VictorSPX rightFollower1;
  private final VictorSPX rightFollower2;
  
  private final Solenoid shifter;

  private final SpeedControllerGroup leftGroup;
  private final SpeedControllerGroup rightGroup;

  private AHRS navX = null;

  private DifferentialDrive drive;
  private DifferentialDriveOdometry odometry;

  public DriveTrainSubsystem() {
    leftMaster = new WPI_TalonSRX(Constants.Motors.kDriveLeftMasterId);
    leftMaster.configFactoryDefault();
    leftFollower1 = new VictorSPX(Constants.Motors.kDriveLeftFollowerId);
    leftFollower1.configFactoryDefault();
    leftFollower2 = new VictorSPX(Constants.Motors.kDriveLeftFollower2Id);
    leftFollower2.configFactoryDefault();
    rightMaster = new WPI_TalonSRX(Constants.Motors.kDriveRightMasterId);
    rightMaster.configFactoryDefault();
    rightFollower1= new VictorSPX(Constants.Motors.kDriveRightFollowerId);
    rightFollower1.configFactoryDefault();
    rightFollower2 = new VictorSPX(Constants.Motors.kDriveRightFollower2Id);
    rightFollower2.configFactoryDefault();
    shifter = new Solenoid(Constants.Solenoids.kShifterSolenoidID);

    rightMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Constants.DriveTrain.kVelocityControlSlot, Constants.DriveTrain.kCANBusConfigTimeoutMS);
    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Constants.DriveTrain.kVelocityControlSlot, Constants.DriveTrain.kCANBusConfigTimeoutMS);
  
    rightMaster.setInverted(false);
    rightFollower1.setInverted(false);
    rightFollower2.setInverted(false);
    leftMaster.setInverted(false);
    leftFollower1.setInverted(false);
    leftFollower2.setInverted(false);

    rightMaster.setSensorPhase(true);
    leftMaster.setSensorPhase(true);

    rightMaster.setNeutralMode(NeutralMode.Brake);
    leftMaster.setNeutralMode(NeutralMode.Brake);

    rightFollower1.follow(rightMaster);
    rightFollower2.follow(rightMaster);
    leftFollower1.follow(leftMaster);
    leftFollower2.follow(leftMaster);

    leftGroup = new SpeedControllerGroup(leftMaster);
    rightGroup  = new SpeedControllerGroup(rightMaster);    
    drive = new DifferentialDrive(leftGroup, rightGroup);

    // try {
    //   navX = new AHRS(SPI.Port.kMXP);
    //   navX.enableLogging(true);
    // } catch (Exception e) {
    //   DriverStation.reportError("Error instantiating navX: ", e.getStackTrace());
    // }

    odometry = new DifferentialDriveOdometry(getAngle());
  }

  public void setOdometry(double x, double y){

    Pose2d newPose = new Pose2d(x, y, getAngle());
      odometry.resetPosition(newPose, getAngle());
  }

  
  public void setHighGear(boolean highGear) {
    shifter.set(!highGear);
  }

  public void arcadeDrive(double tank, double turn) {
    drive.arcadeDrive(tank, turn);
  }

  public Rotation2d  getAngle() {
    if (navX == null) {
      return Rotation2d.fromDegrees(0.0);
    } else {
      return Rotation2d.fromDegrees(navX.getAngle());
    }
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void putToSmartDashboard() {
    SmartDashboard.putNumber("Right Encoder", rightMaster.getSelectedSensorPosition());
    SmartDashboard.putNumber("Left Encoder", leftMaster.getSelectedSensorPosition());
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      //(leftMaster.getSelectedSensorVelocity() / Constants.DriveTrain.kTicksPerRot) * Constants.DriveTrain.kDriveWheelCircumferenceInches * Constants.DriveTrain.kEncoderGearRatio,
      //(rightMaster.getSelectedSensorVelocity() / Constants.DriveTrain.kTicksPerRot) * Constants.DriveTrain.kDriveWheelCircumferenceInches * Constants.DriveTrain.kEncoderGearRatio
    );
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMaster.setVoltage(leftVolts);
    rightMaster.setVoltage(rightVolts);
  }

  public void resetEncoders() {
    rightMaster.setSelectedSensorPosition(0);
    leftMaster.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {
    // odometry.update(
    //   getAngle(),
    //   (leftMaster.getSelectedSensorPosition() / Constants.DriveTrain.kTicksPerRot) * Constants.DriveTrain.kDriveWheelCircumferenceInches * Constants.DriveTrain.kEncoderGearRatio,
    //   (rightMaster.getSelectedSensorPosition() / Constants.DriveTrain.kTicksPerRot) * Constants.DriveTrain.kDriveWheelCircumferenceInches * Constants.DriveTrain.kEncoderGearRatio  
    // );
  }

 
}
