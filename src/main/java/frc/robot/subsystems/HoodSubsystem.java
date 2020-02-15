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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//CIOCCI VENTEREA
//TODO find max and min values/heights for the hood manually with the robot 
//TODO Lemon Switches???

public class HoodSubsystem extends SubsystemBase {
  private TalonSRX angleMotor = null;
  private boolean runningInOpenLoop = true;

  public HoodSubsystem() {
    angleMotor = new TalonSRX(Constants.Motors.kHoodMotorID);
    angleMotor.configFactoryDefault();
    angleMotor.setNeutralMode(NeutralMode.Brake);
    angleMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    angleMotor.configClearPositionOnLimitR(true, 10);

    angleMotor.config_kF(0, 0.00001, 10);
    angleMotor.config_kP(0, 0, 10);
    angleMotor.config_kI(0, 0, 10);
    angleMotor.config_kD(0, 0, 10);
  }

  public int calculateTicksByDistance(double distance) {
    double ticks;
    if (distance < 275){
     ticks = (-0.305212 * distance * distance) + (265.163 * distance) + 74419.9;
    } else {
      ticks = 132311 - (36.85 * distance);
    }

    return (int) ticks;
  }

  public void zeroSensor(){ // to zero hood push it all the way down on the robot
    angleMotor.setSelectedSensorPosition(0, 0, 10);
    m_IsOnTarget = false;
  }

  //Moves hood up at low incaments with a max postion checker
  public void manualMoveHoodUp(){ 
    //TODO check gear ratios to find better motor speed
    System.out.println("Manual UP");
    if (angleMotor.getSelectedSensorPosition() >= Constants.Hood.kMaxHoodTicks) {
      angleMotor.set(ControlMode.PercentOutput, 0);
     } else {
      angleMotor.set(ControlMode.PercentOutput, Constants.Hood.kHoodUpSpeed);
    }
    m_IsOnTarget = false;
  }

  //Moves Hood Down at low incraments with a min postion checker 
  public void manualMoveHoodDown(){ //TODO check gear ratios to find better motor speed
    System.out.println("Manual Down");
    // if (angleMotor.getSelectedSensorPosition() <= Constants.Hood.kMinHoodHeight) {
    //   angleMotor.set(ControlMode.PercentOutput, 0);
    // } else {
      angleMotor.set(ControlMode.PercentOutput, Constants.Hood.kHoodDownSpeed);
    // }
    m_IsOnTarget = false;
  }
  
  //makes the motor stop
  public void manualStopHoodMovement(){
    //System.out.println("MANUAL STOP");
    System.out.println("STOP 1");
    angleMotor.set(ControlMode.PercentOutput, 0);
    m_IsOnTarget = false;
  }
  
  //AimHood creates an endoder value that it calculates 
  //from a given angle and ticks per rotation then moves 
  //the motor to that encoder value baised on max and min constriants   
  // public void aimHood(double targetEncoderValue ) { 
  //   //TODO: I think all the motion magic PID values will need to be set, check documentation
  //   if (angleMotor.getSelectedSensorPosition() <= Constants.Hood.kMinHoodTicks && targetEncoderValue < Constants.Hood.kMinHoodTicks) {
  //     System.out.println("TOO FAR NEGATIVE HOOD");
  //     angleMotor.set(ControlMode.PercentOutput, 0);
  //   } else if (angleMotor.getSelectedSensorPosition() >= Constants.Hood.kMaxHoodTicks && targetEncoderValue > Constants.Hood.kMaxHoodTicks) {
  //     angleMotor.set(ControlMode.PercentOutput, 0);
  //     System.out.println("TOO FAR POSITIVE HOOD");
  //   } else { 
  //     System.out.println("SETTING ANGLE" + targetEncoderValue);
  //     angleMotor.set(ControlMode.Position, targetEncoderValue); 
  //   }
  // }

  public void driveToEncoderPos(double ticks) {
    double currentTicks = angleMotor.getSelectedSensorPosition();
    if (currentTicks <= Constants.Hood.kMinHoodTicks && ticks < Constants.Hood.kMinHoodTicks) {
      System.out.println("HOOD TOO LOW " + currentTicks + " " + ticks);
      angleMotor.set(ControlMode.PercentOutput, 0);
      m_IsOnTarget = false;
    } else if (currentTicks >= Constants.Hood.kMaxHoodTicks && ticks > Constants.Hood.kMaxHoodTicks) {
      System.out.println("HOOD TOO HIGH " + currentTicks + " " + ticks);
      angleMotor.set(ControlMode.PercentOutput, 0);
      m_IsOnTarget = false;
    } else { 

      // double pct = (ticks - currentTicks) /20000.0;
      // if (pct > 1) {
      //   pct = 1;
      // } else if (pct < -1) {
      //   pct = -1;
      // } else if (Math.abs(pct) < .2) {
      //   pct = 0;
      // }
      // double power = pct * .2;

      double difference = ticks - currentTicks;

      double power = 0;
      if (Math.abs(difference) < 5000) {
        System.out.println("ON TARGET");
        power = 0; 
        m_IsOnTarget = true;
      }else {
//        if(Math.abs(difference) > 20000) {
          if (ticks > currentTicks) {
            System.out.println("TRYING TO GO FORWARDS");
            power = .25;
            m_IsOnTarget = false;
          } else if (ticks < currentTicks) {
            System.out.println("TRYING TO GO BACKWARDS");
            power = -.25;
            m_IsOnTarget = false;
          }
        // } else {
        //   m_IsOnTarget = false;
          
        //   power = (difference/20000) * Constants.Hood.kMaxHoodSpeed;
        //   if(power < Constants.Hood.kMinHoodSpeed) {
        //     power = Constants.Hood.kMinHoodSpeed;
        //   }
        // }
      }
      System.out.println("POWER ======" + power);
//      System.out.println("POWERING HOOD " + power + "  Current: " + currentTicks + " Target: " + ticks + " RAW " + angleMotor.getSelectedSensorPosition());
      angleMotor.set(ControlMode.PercentOutput, power);
    }
      // if(Math.abs(ticks - currentTicks) < 1000) {
      //   System.out.println("HOOD CLOSE " + currentTicks + " " + ticks);
      //   angleMotor.set(ControlMode.PercentOutput, 0);
      // } else {
      //   if(currentTicks < ticks) {
      //     System.out.println("HOOD UP " + currentTicks + " " + ticks);
      //     angleMotor.set(ControlMode.PercentOutput, .2);
      //   } else if(currentTicks > ticks) {
      //     System.out.println("HOOD DOWN " + currentTicks + " " + ticks);
      //     angleMotor.set(ControlMode.PercentOutput, -.2);
      //   } else {
      //     System.out.println("HOOD ELSE");
      //     angleMotor.set(ControlMode.PercentOutput, 0);
      //   }
      // }
  }

  private boolean m_IsOnTarget = false;
  public boolean isOnTarget()
  {
    return m_IsOnTarget;
  }

  public void resetEncoder() {
    m_IsOnTarget = false;
    angleMotor.setSelectedSensorPosition(0);
  }

  public void putEncoderToShuffleboard() {
    SmartDashboard.putNumber("Hood encoder", angleMotor.getSelectedSensorPosition());
  }

//getCurrentAngle returns the current angle of the hood
  public double getCurrentTicks() {
    return angleMotor.getSelectedSensorPosition();
    // double currentTicks = angleMotor.getSelectedSensorPosition();
    // double currentAngle = (currentTicks / Constants.Hood.kTicksPerRotation) * 360;
    // return currentAngle;
  }
  
}
