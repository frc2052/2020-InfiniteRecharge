/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//TODO: The elevator will probably go up and down slowly. Confirm speed with build team. All this complexity can probably be simplified if it goes slow. Ask Scott
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

//TODO: probably not needed anymore
public class ClimberSubsystem extends SubsystemBase {  
  private final WPI_TalonSRX climberMotor = new WPI_TalonSRX(Constants.Motors.kClimberMotorID);
  private int goalElevatorInches;
  private boolean runningInOpenLoop = false;
 
  public ClimberSubsystem() {   
    // TODO reset motor controllers to factory defaults
    climberMotor.setNeutralMode(NeutralMode.Brake);
    climberMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    climberMotor.setInverted(true);
    climberMotor.setSensorPhase(true);

    /* Set relevant frame periods to be at least as fast as periodic rate */
    climberMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 10);
    climberMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 10);

    /* set the peak and nominal outputs */
    climberMotor.configNominalOutputForward(0, 10);
    climberMotor.configNominalOutputReverse(0, 10);
    climberMotor.configPeakOutputForward(Constants.Elevator.kPercentOutputUp, 10);
    climberMotor.configPeakOutputReverse(Constants.Elevator.kPercentOutputDown, 10);

    /* set closed loop gains in slot0 - see documentation */
    climberMotor.selectProfileSlot(0, 0);
    climberMotor.config_kF(0, 0.2, 10); //1843
    climberMotor.config_kP(0, .8, 10);
    climberMotor.config_kI(0, 0, 10);
    climberMotor.config_kD(0, 0, 10);

    /* set acceleration and vcruise velocity - see documentation */
    climberMotor.configMotionCruiseVelocity((int)(5550 * .9), 10);
    climberMotor.configMotionAcceleration((int)(5550 * .9 * 2), 10);
    /* zero the sensor */
    climberMotor.setSelectedSensorPosition(0, 0, 10);
  }
  public void ManualUp() {
    climberMotor.set(ControlMode.PercentOutput, Constants.Elevator.kManualPercent);
  }

  public void ManualDown() {
    climberMotor.set(ControlMode.PercentOutput,-Constants.Elevator.kManualPercent);
  }

  public void ManualStop () {
   climberMotor.set(ControlMode.PercentOutput,0);
  }
  public double getHeightInches() {
    int encoderPos = climberMotor.getSelectedSensorPosition(0);
    double revolutions = encoderPos / (double)Constants.Elevator.kElevatorTicksPerRot;
    double inches = revolutions * Constants.Elevator.kElevatorInchesPerRotation;
    return inches;
  }
  public void zeroSensor(){
    climberMotor.setSelectedSensorPosition(0, 0, 10);
  }
  private void setAndVerifyGoalInches(int newGoalInches){
    if (newGoalInches >  Constants.Elevator.kElevatorMaxHeight) {
        goalElevatorInches = Constants.Elevator.kElevatorMaxHeight;
    }
    else if (newGoalInches < Constants.Elevator.kElevatorMinHeight) {
        System.out.println("INVALID ELEVATOR VALUE : " + newGoalInches);
        goalElevatorInches = Constants.Elevator.kElevatorMinHeight;
    }
    else {
        goalElevatorInches = newGoalInches;

    }
    if (goalElevatorInches <= getHeightInches()) {
        climberMotor.configMotionCruiseVelocity((int)(5550 * .5), 10);
        climberMotor.configMotionAcceleration((int)(5550 * .5 * 2), 10);
    }else{
        climberMotor.configMotionCruiseVelocity((int)(5550 * .9), 10);
        climberMotor.configMotionAcceleration((int)(5550 * .9 * 2), 10);
    }
    

  }

  public int getHeightInchesForPreset(ElevatorPresetEnum posEnum){
    switch (posEnum){
        case LOW:
            return Constants.Elevator.kElevatorMinHeight;
        case LEVEL:
            return (Constants.Elevator.kElevatorHeight);
        case HIGH:
            return (int)(Constants.Elevator.kElevatorMaxHeight);
    }
    return 0;
  }

    public void setTarget(ElevatorPresetEnum posEnum) {
    //sets goal to the correct inches according to the preset
    runningInOpenLoop = false;
    int calcTarget = getHeightInchesForPreset(posEnum);
    setAndVerifyGoalInches(calcTarget);
  }
  @Override
  public void periodic() 
  {
    if(getHeightInches() < 0){
        zeroSensor();
    }
    if(!runningInOpenLoop) {//we are running in closed loop
      double rotation = goalElevatorInches / Constants.Elevator.kElevatorInchesPerRotation;
      int pos = (int) (rotation * Constants.Elevator.kElevatorTicksPerRot);
      //Sets the Carriage at a set height, see https://github.com/CrossTheRoadElec/Phoenix-Documentation/blob/master/Talon%20SRX%20Victor%20SPX%20-%20Software%20Reference%20Manual.pdf
      // in 3.1.2.1, recommended timeout is zero while in robot loop
      climberMotor.set(ControlMode.MotionMagic, pos);
      SmartDashboard.putNumber("ElevatorTargetPos", goalElevatorInches);
      SmartDashboard.putNumber("ElevatorPos", getHeightInches());
    }
    super.periodic();
  }
  public enum ElevatorPresetEnum {
    LOW,
    LEVEL, 
    HIGH, 
  }

  
}