/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Constants;

public class HoodSubsystem extends SubsystemBase {
  private TalonSRX angleMotor = new TalonSRX(Constants.Shooter.kAngleMotorID);
  private int goalAngleHeight = 0;
  private boolean runningInOpenLoop = false;

  public HoodSubsystem() {
    angleMotor.setNeutralMode(NeutralMode.Brake);
    angleMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    
  }

  public void zeroSensor(){

    angleMotor.setSelectedSensorPosition(0, 0, 10);
}

  public void setTarget(anglePresetEnum posEnum) {

    //sets goal to the correct inches according to the preset

    runningInOpenLoop = false;

    int calcTarget = getAngleHeight(posEnum);

    setAndVerifyGoalInches(calcTarget);

}

public double getHeightInches() {

  int encoderPos = angleMotor.getSelectedSensorPosition(0);

  double revolutions = encoderPos / (double)Constants.Shooter.kAngleTicsPerRotations;
  double targetPos = revolutions * Constants.Shooter.kInchesPerRotation;

  return targetPos;

}

public void setCurrentPosAsTarget(){

    setAndVerifyGoalInches((int)getHeightInches());

}

private void setAndVerifyGoalInches(int newGoalInches){

    if (newGoalInches >  Constants.Shooter.kMaxAngleHeight) {

      goalAngleHeight = Constants.Shooter.kMaxAngleHeight;

    }

    else if (newGoalInches < Constants.Shooter.kMinAngleHeight) {

        System.out.println("INVALID ELEVATOR VALUE : " + newGoalInches);

        goalAngleHeight = Constants.Shooter.kMinAngleHeight;

    }

    else {

        goalAngleHeight = newGoalInches;

    }
  }


private boolean emergencyDownWasPressed = false; // variable makes it able to stop the motor only one time once it is let go

  public void StartEmergencyDown() {

    angleMotor.set(ControlMode.PercentOutput,Constants.Shooter.kEmergencyDownPower);

}

  public void StopEmergencyMove(){
    angleMotor.set(ControlMode.PercentOutput, Constants.Shooter.kEmergencyHoldPower);

  }

  public void starEmergencyUp(){
    angleMotor.set(ControlMode.PercentOutput, Constants.Shooter.kEmergencyUpPower);
  }

  public void setAngleAdjustmentUp(){
    setAndVerifyGoalInches(goalAngleHeight + 10);
  }

  public void setAngleAdjustmentDown(){
    setAndVerifyGoalInches(goalAngleHeight -10);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(getHeightInches() == 0){
        zeroSensor();
    }
    double rotation = goalAngleHeight/Constants.Shooter.kAngleTicsPerRotations;
    int pos = (int) (rotation * Constants.Shooter.kAngleTicsPerRotations);
    angleMotor.set(ControlMode.MotionMagic, pos);
  }

  public int getAngleHeight(anglePresetEnum anglePreset) {
    switch(anglePreset){
      case CLOSE:
        return  Constants.Shooter.kCloseAnglePosition;
      case MIDDLE:
        return Constants.Shooter.kMiddleAnglePosition;
      case FAR:
        return Constants.Shooter.kFarAnglePosition;
    }

    return 0; 
  }

  public enum anglePresetEnum{
    CLOSE(),
    MIDDLE,
    FAR,
  }

}
