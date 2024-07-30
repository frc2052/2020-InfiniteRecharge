package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Compressor;
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
  private final Compressor compressor;

    public IntakeSubsystem() {
        upSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Solenoids.kUpIntakeSolenoidID);
        downSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Solenoids.kDownIntakeSolenoidID);
        outerIntakeMotor = new VictorSPX(Constants.Motors.kOuterIntakeMotorID);
        outerIntakeMotor.configFactoryDefault();

        compressor = new Compressor(PneumaticsModuleType.CTREPCM);
        compressor.enableDigital();

        outerIntakeMotor.setNeutralMode(NeutralMode.Coast);
    }

    public void armUp(){
        System.out.println("-----ARM GOING UP");
        downSolenoid.set(false);
        upSolenoid.set(true);
    }
    public void armDown(){
        System.out.println("-----ARM GOING DOWN");
        upSolenoid.set(true);
        downSolenoid.set(false);
    }
    public void intakeIn(){
        outerIntakeMotor.set(ControlMode.PercentOutput, Constants.Intake.kIntakeSpeed);
        intakePct = Constants.Intake.kIntakeSpeed;
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

    @Override
    public void periodic() {
    }
        
}