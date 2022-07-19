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

    // private Joystick turnJoystick;
    // private Joystick tankJoystick;
    // private Joystick secondaryPanel;

    // public ShooterControls(Joystick turn, Joystick tank, Joystick secondary) {
    //     turnJoystick = turn;
    //     tankJoystick = tank;
    //     secondaryPanel = secondary;
    // }

    private Joystick primaryJoystick;

    public ShooterControls(Joystick primaryJoystick) {
        this.primaryJoystick = primaryJoystick;
    }

    public boolean getShooterIncrease() {
        // return secondaryPanel.getY() < -.5;
        return primaryJoystick.getRawButton(5);
    }

    public boolean getShooterDecrease() {
        // return secondaryPanel.getY() > .5;
        return primaryJoystick.getRawButton(3);
    }

    public boolean getManualHoodUp() {
        return primaryJoystick.getPOV() == 0;
    }

    public boolean getManualHoodDown() {
        return primaryJoystick.getPOV() == 180; 
    }

    public boolean getManualTurretLeft() {
        return primaryJoystick.getPOV() == 270;
    }

    public boolean getManualTurretRight() {
        return primaryJoystick.getPOV() == 90;
    }

    public boolean getReadyPressed() {
        return primaryJoystick.getRawButton(12);
    }

    public boolean getShootPressed() {
        return primaryJoystick.getTrigger();
    }

    public boolean getManualConveyorDown() {
        // return secondaryPanel.getX() > .5;
        return false;
    }

    public boolean getManualConveyorUp() {
        // return secondaryPanel.getX() < -.5;
        return false;
    }

    public boolean getLoadConveyor() {
        // return secondaryPanel.getRawButton(4);
        return false;
    }

    public boolean getIdleShooterToggle() {
        // System.out.println("SHOOTER IDLE TOGGLE =" + tankJoystick.getRawButton(4));
        return primaryJoystick.getRawButton(4);
        //return false;
    }
}
