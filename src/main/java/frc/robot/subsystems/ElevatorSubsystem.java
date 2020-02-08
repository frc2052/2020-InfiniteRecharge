package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
    private final WPI_TalonSRX climberMotor;
    private Solenoid inSolenoid;
    private Solenoid outSolenoid;

    public ElevatorSubsystem() {
        climberMotor = new WPI_TalonSRX(Constants.Motors.kClimberMotorID);

        climberMotor.setNeutralMode(NeutralMode.Brake);
        climberMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
        climberMotor.setInverted(true);
        climberMotor.setSelectedSensorPosition(0, 0, 10);

        inSolenoid = new Solenoid(Constants.Solenoids.kInIntakeSolenoidID);
        outSolenoid = new Solenoid(Constants.Solenoids.kOutIntakeSolenoidID);    
    }

    private boolean isOverride;

    public void setOverride(boolean isPressed)
    {
        isOverride = isPressed;
    }

    public void pistonOut() {
        inSolenoid.set(false);
        outSolenoid.set(true);
    }

    public void pistonIn() {
        inSolenoid.set(true);
        outSolenoid.set(false);
    }

    public void manualUp() {
        pistonIn();
        if(isOverride == false) {
            double currentHeight = this.getHeightInches();
            if (currentHeight < Constants.Elevator.kElevatorMaxHeight) {
                climberMotor.set(ControlMode.PercentOutput, .2);
            }
            else {
                climberMotor.set(ControlMode.PercentOutput, 0);
            }
        } else {
            climberMotor.set(ControlMode.PercentOutput, .2);
        }
    }
   
    public void manualDown() {
        pistonIn();
        if(isOverride == false) {
            double currentHeight = this.getHeightInches();
            if (currentHeight > Constants.Elevator.kElevatorMinHeight)
            {
                climberMotor.set(ControlMode.PercentOutput, -.2);
            }
            else 
            {
                climberMotor.set(ControlMode.PercentOutput, 0);
            }
        } else {
            climberMotor.set(ControlMode.PercentOutput, -.2);
        }
    }

    public void manualStop() {
        climberMotor.set(ControlMode.PercentOutput, 0);
        pistonOut();
    }

    public double getHeightInches() {
        int encoderPos = climberMotor.getSelectedSensorPosition(0);
        double revolutions = encoderPos / (double) Constants.Elevator.kElevatorTicksPerRot;
        double inches = revolutions * Constants.Elevator.kElevatorInchesPerRotation;
        return inches;
    }

    // private void setAndVerifyGoalInches(int newGoalInches) {
    //     if (newGoalInches > Constants.Elevator.kElevatorMaxHeight) {
    //     } else if (newGoalInches < Constants.Elevator.kElevatorMinHeight) {
    //         System.out.println("INVALID ELEVATOR VALUE : " + newGoalInches);
    //     } else {
    //     }

    // }

    // public int getHeightInchesForPreset(ElevatorPresetEnum posEnum) {
    //     switch (posEnum) {
    //     case LOW:
    //         return Constants.Elevator.kElevatorMinHeight;
    //     case LEVEL:
    //         return (Constants.Elevator.kElevatorHeight);
    //     case HIGH:
    //         return (int) (Constants.Elevator.kElevatorMaxHeight);
    //     }
    //     return 0;
    // }

    // public void setTarget(ElevatorPresetEnum posEnum) {
    //     int calcTarget = getHeightInchesForPreset(posEnum);
    //             setAndVerifyGoalInches(calcTarget);
    //         }
}