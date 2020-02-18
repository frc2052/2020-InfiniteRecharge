/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;

import frc.robot.IShooterControls;

public class AutoShooterControls implements IShooterControls {

    @Override
    public boolean getManualConveyorDown() {
        return false;
    }

    @Override
    public boolean getManualConveyorUp() {
        return false;
    }

    @Override
    public boolean getManualHoodDown() {
        return false;
    }

    @Override
    public boolean getManualHoodUp() {
        return false;
    }

    @Override
    public boolean getManualTurretLeft() {
        return false;
    }

    @Override
    public boolean getManualTurretRight() {
        return false;
    }

    @Override
    public boolean getReadyPressed() {
        return false;
    }

    private boolean isShootPressed = true;

    @Override
    public boolean getShootPressed() {
        return isShootPressed;
    }

    public void setShootPressed(boolean isPressed){
        isShootPressed = isPressed;
    }


    @Override
    public boolean getShooterDecrease() {
        return false;
    }

    @Override
    public boolean getShooterIncrease() {
        return false;
    }
    
}
