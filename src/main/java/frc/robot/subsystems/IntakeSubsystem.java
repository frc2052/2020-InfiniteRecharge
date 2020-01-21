package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Solenoid;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
    private Solenoid upSolenoid = new Solenoid(Constants.Intake.kUpIntakeSolenoidID);
    private Solenoid downSolenoid = new Solenoid(Constants.Intake.kDownIntakeSolenoidID);
    private VictorSPX outerIntakeMotor = new VictorSPX(Constants.Intake.kOuterIntakeMotorID);
    private boolean armActive;
    public IntakeSubsystem() {

    }
    public void armToggle(){
        if(armActive){
            armUp();
        }
        else{
            armDown();
        }
    }
    public void armUp(){
        downSolenoid.set(false);
        upSolenoid.set(true);
        armActive = false;
    }
    public void armDown(){
        upSolenoid.set(false);
        downSolenoid.set(true);
        armActive = true;
    }
    public void intakeIn(){
        outerIntakeMotor.set(ControlMode.PercentOutput, 1);
    }
    public void intakeOut(){
        outerIntakeMotor.set(ControlMode.PercentOutput, -1);
    }
    public void intakeStop(){
        outerIntakeMotor.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void periodic() {
    // This method will be called once per scheduler run
    }
}