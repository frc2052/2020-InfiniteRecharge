package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  private final WPI_TalonSRX climberMotor;
  private Solenoid lockinSolenoid;
  private Solenoid lockoutSolenoid;
  private boolean isOverride = false;
  private boolean isLocked = false;

  public ElevatorSubsystem() {
      climberMotor = new WPI_TalonSRX(Constants.Motors.kClimberMotorID);
      climberMotor.setNeutralMode(NeutralMode.Brake);
      climberMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
      climberMotor.setInverted(false);
      climberMotor.setSelectedSensorPosition(0, 0, 10);

      lockinSolenoid = new Solenoid(Constants.Solenoids.kElevatorLockSolenoidID);
      lockoutSolenoid = new Solenoid(Constants.Solenoids.kElevatorUnLockSolenoidID);
  }

    public void setOverride(boolean isPressed) {
        isOverride = isPressed;
    }

    public void lockElevator(){
        isLocked = true;
        lockinSolenoid.set(false);
        lockoutSolenoid.set(true);
    }   

    public void unlockElevator(){
        isLocked = false;
        lockinSolenoid.set(true);
        lockoutSolenoid.set(false);
    }

    public void printEncoderPos() {
        SmartDashboard.putNumber("Elevator Encoder Pos", climberMotor.getSelectedSensorPosition());
    }

    public void manualUp() {
        //System.out.print("Elevator Height: " + this.getHeightInches());
        double currentHeight = climberMotor.getSelectedSensorPosition();
        if (isLocked) {
            climberMotor.set(ControlMode.PercentOutput, 0); //not allowed to drive if lock engaged
            System.out.println("Elevator is locked");
        } else if (currentHeight < Constants.Elevator.kElevatorMaxHeight || isOverride){
            System.out.println("Going Up");
            climberMotor.set(ControlMode.PercentOutput, .7);
        } else {
            System.out.println("WENT TOO FAR" + climberMotor.getSelectedSensorPosition() + "MAX HEIGHT" + Constants.Elevator.kElevatorMaxHeight);
            climberMotor.set(ControlMode.PercentOutput, 0);
        }
    }
   
    public void manualDown() {
        System.out.print("Elevator Height: " + this.getHeightInches());
        double currentHeight = climberMotor.getSelectedSensorPosition(0);;
        if (isLocked) {
            System.out.println("Elevator Locked");;
            climberMotor.set(ControlMode.PercentOutput, 0); //not allowed to go down if lock engaged
        }
        else if (currentHeight > Constants.Elevator.kElevatorMinHeight || isOverride)
        {
            System.out.println("Elevator Going Down");;
            climberMotor.set(ControlMode.PercentOutput, -.7);
        }
        else 
        {
            System.out.println("Elevator Stop");;
            climberMotor.set(ControlMode.PercentOutput, 0);
        }
    }

    public void resetEncoder() {
        climberMotor.setSelectedSensorPosition(0);
    }

    public void manualStop() {
        climberMotor.set(ControlMode.PercentOutput, 0);
    }

    public double getHeightInches() {
        int encoderPos = climberMotor.getSelectedSensorPosition(0);
        double revolutions = encoderPos / (double) Constants.Elevator.kElevatorTicksPerRot;
        double inches = revolutions * Constants.Elevator.kElevatorInchesPerRotation;
        return inches;
    }    
}