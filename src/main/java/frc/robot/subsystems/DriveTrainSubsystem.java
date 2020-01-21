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
import frc.robot.Constants;

public class DriveTrainSubsystem extends SubsystemBase {

  private final WPI_TalonSRX leftMaster = new WPI_TalonSRX(Constants.DriveTrain.kDriveLeftMasterId);
  private final VictorSPX leftSlave1 = new VictorSPX(Constants.DriveTrain.kDriveLeftSlaveId);
  private final VictorSPX leftSlave2 = new VictorSPX(Constants.DriveTrain.kDriveLeftSlave2Id);
  
  private final WPI_TalonSRX rightMaster = new WPI_TalonSRX(Constants.DriveTrain.kDriveRightMasterId);
  private final VictorSPX rightSlave1 = new VictorSPX(Constants.DriveTrain.kDriveRightSlaveId);
  private final VictorSPX rightSlave2 = new VictorSPX(Constants.DriveTrain.kDriveRightSlave2Id);
  
  private final Solenoid shifterIn = new Solenoid(Constants.DriveTrain.kShiftInSolenoidID);
  private final Solenoid shifterOut = new Solenoid(Constants.DriveTrain.kShiftOutSolenoidID);

  private final SpeedControllerGroup leftGroup = new SpeedControllerGroup(leftMaster);
  private final SpeedControllerGroup rightGroup = new SpeedControllerGroup(rightMaster);

  private AHRS navX = null;

  private DifferentialDrive drive = new DifferentialDrive(leftGroup, rightGroup);
  private DifferentialDriveOdometry odometry;

  public DriveTrainSubsystem() {
    leftMaster.configFactoryDefault();
    leftSlave1.configFactoryDefault();
    leftSlave2.configFactoryDefault();
    rightMaster.configFactoryDefault();
    rightSlave1.configFactoryDefault();
    rightSlave2.configFactoryDefault();

    rightMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Constants.DriveTrain.kVelocityControlSlot, Constants.DriveTrain.kCANBusConfigTimeoutMS);
    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Constants.DriveTrain.kVelocityControlSlot, Constants.DriveTrain.kCANBusConfigTimeoutMS);
  
    rightMaster.setInverted(false);
    rightSlave1.setInverted(false);
    rightSlave2.setInverted(false);
    leftMaster.setInverted(true);
    leftSlave1.setInverted(true);
    leftSlave2.setInverted(true);

    rightMaster.setSensorPhase(true);
    leftMaster.setSensorPhase(true);

    rightMaster.setNeutralMode(NeutralMode.Brake);
    leftMaster.setNeutralMode(NeutralMode.Brake);

    rightSlave1.follow(rightMaster);
    rightSlave2.follow(rightMaster);
    leftSlave1.follow(leftMaster);
    leftSlave2.follow(leftMaster);
    
    try {
      navX = new AHRS(SPI.Port.kMXP);
      navX.enableLogging(true);
    } catch (Exception e) {
      DriverStation.reportError("Error instantiating navX: ", e.getStackTrace());
    }

    odometry = new DifferentialDriveOdometry(getAngle());
  }

  
  public void setHighGear(boolean highGear) {
    shifterOut.set(!highGear);
    shifterIn.set(highGear);
  }

  public void tankDrive(double left, double right) {
    drive.tankDrive(left, right);
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

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      (leftMaster.getSelectedSensorVelocity() / Constants.DriveTrain.kTicksPerRot) * Constants.DriveTrain.kDriveWheelCircumferenceInches * Constants.DriveTrain.kEncoderGearRatio,
      (rightMaster.getSelectedSensorVelocity() / Constants.DriveTrain.kTicksPerRot) * Constants.DriveTrain.kDriveWheelCircumferenceInches * Constants.DriveTrain.kEncoderGearRatio
    );
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMaster.setVoltage(leftVolts);
    rightMaster.setVoltage(rightVolts);
  }

  @Override
  public void periodic() {
    odometry.update(
      getAngle(),
      (leftMaster.getSelectedSensorPosition() / Constants.DriveTrain.kTicksPerRot) * Constants.DriveTrain.kDriveWheelCircumferenceInches * Constants.DriveTrain.kEncoderGearRatio,
      (rightMaster.getSelectedSensorPosition() / Constants.DriveTrain.kTicksPerRot) * Constants.DriveTrain.kDriveWheelCircumferenceInches * Constants.DriveTrain.kEncoderGearRatio  
    );
  }

 
}
