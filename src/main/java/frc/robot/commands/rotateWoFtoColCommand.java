package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WheelOfFortuneSubsystem;


public class rotateWoFtoColCommand extends CommandBase {
    private final WheelOfFortuneSubsystem wheelOfFortuneSubsystem;

    public rotateWoFtoColCommand(WheelOfFortuneSubsystem wheelOfFortuneSubsystem) {
        this.wheelOfFortuneSubsystem = wheelOfFortuneSubsystem;
        addRequirements(wheelOfFortuneSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        wheelOfFortuneSubsystem.turnWheelRight();
    }

    @Override
    public boolean isFinished() {
        if(checkColorMatch(wheelOfFortuneSubsystem.getColorSensor())){
            return true;
        }
        else{
            return false;
        }

    }

    @Override
    public void end(boolean interrupted) {

    }

    private String m_gameData = null;
    private String getGameData() {
        if (m_gameData == null || m_gameData.length() == 0) {
            m_gameData = DriverStation.getInstance().getGameSpecificMessage();
        }
        return m_gameData;
    }


    private boolean checkColorMatch(Color colorSensed)
    {
        String gameData = getGameData();
        if(gameData != null && gameData.length() > 0) {
            switch (gameData.charAt(0)) {
                case 'B' :
                    return (colorSensed.blue > .9 && colorSensed.red < .1 && colorSensed.green < .1);
                case 'G' :
                    return (colorSensed.green > .9 && colorSensed.blue < .1 && colorSensed.red < .1);
                case 'R' :
                    return (colorSensed.red > .9 && colorSensed.blue < .1 && colorSensed.green < .1);

                case 'Y' :
                    return (colorSensed.red > .9 && colorSensed.green > 0.9 && colorSensed.blue < .1);

                default :
                    return false;

            }
        } else {
            return false;
        }
    }


}