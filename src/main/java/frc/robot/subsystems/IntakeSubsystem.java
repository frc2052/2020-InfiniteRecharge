package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  private Solenoid upSolenoid;
  private Solenoid downSolenoid;
  private VictorSPX outerIntakeMotor;
  private boolean isArmDown;
  private double intakePct;

    public IntakeSubsystem() {
        upSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.Solenoids.kUpIntakeSolenoidID);
        downSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.Solenoids.kDownIntakeSolenoidID);
        outerIntakeMotor = new VictorSPX(Constants.Motors.kOuterIntakeMotorID);
        outerIntakeMotor.configFactoryDefault();

        outerIntakeMotor.setNeutralMode(NeutralMode.Coast);
    }
    public void armToggle(){
        if(isArmDown){
            System.out.println("-----ARM GOING UP");
            armUp();
        } else {
            System.out.println("-----ARM GOING DOWN");
            armDown();
        }
    }
    public void armUp(){
        downSolenoid.set(true);
        upSolenoid.set(false);
        isArmDown = false;
    }
    public void armDown(){
        upSolenoid.set(true);
        downSolenoid.set(false);
        isArmDown = true;
    }
    public void intakeIn(){
        outerIntakeMotor.set(ControlMode.PercentOutput, Constants.Intake.kIntakeSpeed);
        intakePct = Constants.Intake.kIntakeSpeed;
    }

    public boolean getIsArmDown() {
        return isArmDown;
    }

    public void intakeOut(){
        outerIntakeMotor.set(ControlMode.PercentOutput, -Constants.Intake.kIntakeSpeed);
        intakePct = -Constants.Intake.kIntakeSpeed;
    }

    public void intakeStop(){
        outerIntakeMotor.set(ControlMode.PercentOutput, 0);
        intakePct = 0;
    }

    public double getIntakeSpeed() {
        return intakePct;
    }
}