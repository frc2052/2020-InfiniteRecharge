package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.I2C;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WheelOfFortune;
import com.revrobotics.ColorSensorV3;


public class WheelOfFortuneSubsystem extends SubsystemBase {

    private TalonSRX  mainMotor;
    private ColorSensorV3 colSense;


    private WheelOfFortuneSubsystem() {
      mainMotor = new TalonSRX(WheelOfFortune.kControlPanelID);
      mainMotor.configFactoryDefault();
      mainMotor.setNeutralMode(NeutralMode.Brake);

      colSense = new ColorSensorV3(I2C.Port.kOnboard);

      //TODO: Configure Color Sensor
      //colSense.configureColorSensor(ColorSensorV3.ColorSensorResolution.kColorSensorRes16bit);
    }

    public void turnWheelRight() {
        mainMotor.set(ControlMode.PercentOutput, -WheelOfFortune.kMotorPower);
    }

    public void turnWheelLeft() {
        mainMotor.set(ControlMode.PercentOutput, WheelOfFortune.kMotorPower);
    }
    public void wheelStop(){
        mainMotor.set(ControlMode.PercentOutput, 0.0);
    }

    public Color getColorSensor() {
        return colSense.getColor();
    }

}
