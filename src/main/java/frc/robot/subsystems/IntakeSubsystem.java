package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import frc.robot.Constants;
import frc.robot.Constants.ConveyorSubsystem;
import frc.robot.lib.CsvLogger;

public class IntakeSubsystem extends SubsystemBase {
  private Solenoid upSolenoid;
  private Solenoid downSolenoid;
  private VictorSPX outerIntakeMotor;
  private boolean isArmDown;
  private double intakePct;

    public IntakeSubsystem() {
        upSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Solenoids.kUpIntakeSolenoidID);
        downSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Solenoids.kDownIntakeSolenoidID);
        outerIntakeMotor = new VictorSPX(Constants.Motors.kOuterIntakeMotorID);
        outerIntakeMotor.configFactoryDefault();

        outerIntakeMotor.setNeutralMode(NeutralMode.Coast);

        CsvLogger.addLoggingFieldDouble("Intake speed", "", "getIntakeSpeed", this);
        CsvLogger.addLoggingFieldBoolean("is arm down", "", "getIsArmDown", this);
    }
    public void armToggle(){
        //System.out.println("ARM TOGGLE");
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
        //System.out.println("INATKE IN");
        outerIntakeMotor.set(ControlMode.PercentOutput, Constants.Intake.kIntakeSpeed);
        intakePct =  Constants.Intake.kIntakeSpeed;
    }

    public boolean getIsArmDown() {
        return isArmDown;
    }

    

    public void intakeOut(){
        //System.out.println("INATKE OUT");
        outerIntakeMotor.set(ControlMode.PercentOutput, -Constants.Intake.kIntakeSpeed);
        intakePct = -Constants.Intake.kIntakeSpeed;
    }
    public void intakeStop(){
        //System.out.println("INATKE STOP");
        outerIntakeMotor.set(ControlMode.PercentOutput, 0);
        intakePct = 0;
    }

    public double getIntakeSpeed() {
        return intakePct;
    }

}