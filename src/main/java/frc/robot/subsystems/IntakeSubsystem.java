package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Solenoid;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import frc.robot.Constants;
import frc.robot.Constants.ConveyorSubsystem;

public class IntakeSubsystem extends SubsystemBase {
  private Solenoid upSolenoid;
  private Solenoid downSolenoid;
  private VictorSPX outerIntakeMotor;
  private boolean isArmDown;

    public IntakeSubsystem() {
        upSolenoid = new Solenoid(Constants.Intake.kUpIntakeSolenoidID);
        downSolenoid = new Solenoid(Constants.Intake.kDownIntakeSolenoidID);
        outerIntakeMotor = new VictorSPX(Constants.Intake.kOuterIntakeMotorID);
        outerIntakeMotor.configFactoryDefault();
    }
    public void armToggle(){
        if(isArmDown){
            armUp();
        } else {
            armDown();
        }
    }
    public void armUp(){
        downSolenoid.set(false);
        upSolenoid.set(true);
        isArmDown = false;
    }
    public void armDown(){
        upSolenoid.set(false);
        downSolenoid.set(true);
        isArmDown = true;
    }
    public void intakeIn(){
        outerIntakeMotor.set(ControlMode.PercentOutput, Constants.Intake.kIntakeSpeed);
    }
    public void intakeOut(){
        outerIntakeMotor.set(ControlMode.PercentOutput, -Constants.Intake.kIntakeSpeed);
    }
    public void intakeStop(){
        outerIntakeMotor.set(ControlMode.PercentOutput, 0);
    }

}