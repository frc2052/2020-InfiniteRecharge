/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


public class ShooterControls implements IShooterControls{

    private Joystick turnJoystick;
    private Joystick tankJoystick;
    private Joystick secondaryPanel;

    public ShooterControls(Joystick turn, Joystick tank, Joystick secondary) {
        turnJoystick = turn;
        tankJoystick = tank;
        secondaryPanel = secondary;
    }

    public boolean getShooterIncrease() {
        return secondaryPanel.getY() < -.5;
    }

    public boolean getShooterDecrease() {
        return secondaryPanel.getY() > .5;
    }

    public boolean getManualHoodUp() {
        return secondaryPanel.getRawButton(2);
    }

    public boolean getManualHoodDown() {
        return secondaryPanel.getRawButton(8);
    }

    public boolean getManualTurretLeft() {
        return secondaryPanel.getRawButton(9);
    }

    public boolean getManualTurretRight() {
        return secondaryPanel.getRawButton(10);
    }

    public boolean getReadyPressed() {
        return tankJoystick.getRawButton(3);
    }

    public boolean getShootPressed() {
        return tankJoystick.getTrigger();
    }

    public boolean getManualConveyorDown() {
        return secondaryPanel.getX() > .5;
    }


    public boolean getManualConveyorUp() {
        return secondaryPanel.getX() < -.5;
    }

    public boolean getLoadConveyor() {
        return secondaryPanel.getRawButton(4);
    }

    public boolean getIdleShooterToggle() {
        return tankJoystick.getRawButton(4);
    }

}
