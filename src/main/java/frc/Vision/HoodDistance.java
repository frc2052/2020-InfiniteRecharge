/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.vision;

/**
 * Add your docs here.
 */ 
public class HoodDistance {
    private int m_Inches;
    private int m_HoodTicks;
    private double m_ShooterPct;
    
    public HoodDistance(int inches, int hoodTicks, double shooterPct){
        m_Inches = inches;
        m_HoodTicks = hoodTicks;
        m_ShooterPct = shooterPct;
    }
    public int getInches(){
        return m_Inches;
    }
    public int getHoodTicks(){
        return m_HoodTicks;
    }
    public double getShooterPct() {
        return m_ShooterPct;
    }
}
