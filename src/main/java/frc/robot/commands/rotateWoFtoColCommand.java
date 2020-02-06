package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WheelOfFortuneSubsystem;


public class rotateWoFtoColCommand extends CommandBase {
    private final WheelOfFortuneSubsystem wheelOfFortuneSubsystem;
    private Color selectedcol;

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
        if(gameReqCol() == wheelOfFortuneSubsystem.getColorSensor()){
            return true;
        }
        else{
            return false;
        }

    }

    @Override
    public void end(boolean interrupted) {

    }

    public Color gameReqCol(){
        String gameData;
        gameData = DriverStation.getInstance().getGameSpecificMessage();
        if(gameData.length() > 0)
        {
            switch (gameData.charAt(0))
            {
                case 'B' :
                    return new Color(0.0, 1.0, 1.0);
                case 'G' :
                    return new Color(0.0, 1.0, 0.0);

                case 'R' :
                    return new Color(1.0, 0.0, 0.0);

                case 'Y' :
                    return new Color(1.0, 1.0, 0.0);

                default :
                    return wheelOfFortuneSubsystem.getColorSensor();

            }
        } else {
            return wheelOfFortuneSubsystem.getColorSensor();

        }

    }
}
