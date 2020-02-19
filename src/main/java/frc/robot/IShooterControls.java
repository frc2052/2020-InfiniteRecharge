/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Add your docs here.
 */
public interface IShooterControls {
    boolean getShooterIncrease();

    boolean getShooterDecrease();

    boolean getManualHoodUp();

    boolean getManualHoodDown();
    
    boolean getManualTurretLeft();

    public boolean getManualTurretRight();

    public boolean getReadyPressed();

    public boolean getShootPressed();

    public boolean getManualConveyorDown();

    public boolean getManualConveyorUp();

    public boolean getIdleShooterToggle();
}
