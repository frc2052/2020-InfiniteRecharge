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
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import frc.robot.Constants;

public class DriveTrainSubsystem extends SubsystemBase {
  
  private WPI_TalonSRX leftMaster;
  private VictorSPX leftFollower1; 
  private VictorSPX leftFollower2;
  
  private final WPI_TalonSRX rightMaster;
  private final VictorSPX rightFollower1;
  private final VictorSPX rightFollower2;

  private final Solenoid shifter;

  private boolean isHighGear;

  private DifferentialDrive drive;

  public DriveTrainSubsystem() {

    leftMaster = new WPI_TalonSRX(Constants.Motors.kDriveLeftMasterId);
    leftMaster.configFactoryDefault();
    leftFollower1 = new WPI_VictorSPX(Constants.Motors.kDriveLeftFollower1Id);
    leftFollower1.configFactoryDefault();
    leftFollower2 = new WPI_VictorSPX(Constants.Motors.kDriveLeftFollower2Id);
    leftFollower2.configFactoryDefault();
    rightMaster = new WPI_TalonSRX(Constants.Motors.kDriveRightMasterId);
    rightMaster.configFactoryDefault();
    rightFollower1= new WPI_VictorSPX(Constants.Motors.kDriveRightFollower1Id);
    rightFollower1.configFactoryDefault();
    rightFollower2 = new WPI_VictorSPX(Constants.Motors.kDriveRightFollower2Id);
    rightFollower2.configFactoryDefault();
    
    shifter = new Solenoid(PneumaticsModuleType.REVPH, Constants.Solenoids.kShifterSolenoidID);

    rightMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Constants.DriveTrain.kVelocityControlSlot, Constants.DriveTrain.kCANBusConfigTimeoutMS);
    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Constants.DriveTrain.kVelocityControlSlot, Constants.DriveTrain.kCANBusConfigTimeoutMS);
  
    rightMaster.setInverted(false);
    rightFollower1.setInverted(false);
    rightFollower2.setInverted(false);
    leftMaster.setInverted(false);
    leftFollower1.setInverted(false);
    leftFollower2.setInverted(false);

    rightMaster.setSensorPhase(true);
    leftMaster.setSensorPhase(false);

    rightMaster.setNeutralMode(NeutralMode.Brake);
    leftMaster.setNeutralMode(NeutralMode.Brake);
    leftFollower1.setNeutralMode(NeutralMode.Brake);
    leftFollower2.setNeutralMode(NeutralMode.Brake);
    rightFollower1.setNeutralMode(NeutralMode.Brake);
    rightFollower2.setNeutralMode(NeutralMode.Brake);

    rightFollower1.follow(rightMaster);
    rightFollower2.follow(rightMaster);
    leftFollower1.follow(leftMaster);
    leftFollower2.follow(leftMaster);

    drive = new DifferentialDrive(leftMaster, rightMaster);
  }

  public void setHighGear(boolean highGear) {
    shifter.set(highGear);
    isHighGear = highGear;
  }

  public boolean getIsHighGear() {
    return isHighGear;
  }

  public void arcadeDrive(double tank, double turn) {
    drive.arcadeDrive(tank, turn);
  }

  public void curvatureDrive(double tank, double turn, boolean quickTurn) {
    drive.curvatureDrive(tank, turn, quickTurn);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    double leftSpeed = ((double)leftMaster.getSelectedSensorVelocity() / Constants.DriveTrain.kTicksPerRot) * Constants.DriveTrain.kDriveWheelCircumferenceMeters * Constants.DriveTrain.kEncoderGearRatio;
    double rightSpeed = ((double)rightMaster.getSelectedSensorVelocity() / Constants.DriveTrain.kTicksPerRot) * Constants.DriveTrain.kDriveWheelCircumferenceMeters * Constants.DriveTrain.kEncoderGearRatio;


    System.out.println("-----LEFT SPEED: " + leftSpeed + " RIGHT SPEED" + rightSpeed);

    return new DifferentialDriveWheelSpeeds(
      leftSpeed,
      rightSpeed
    );
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMaster.setVoltage(-leftVolts);
    rightMaster.setVoltage(rightVolts);
    drive.feed();
  }
  

  public void resetEncoders() {
    rightMaster.setSelectedSensorPosition(0);
    leftMaster.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {}
}