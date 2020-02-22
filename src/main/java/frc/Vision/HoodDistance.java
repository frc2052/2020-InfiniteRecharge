/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.Vision;

/**
 * Add your docs here.
 */
public class HoodDistance {
    private int m_Inches;
    private int m_HoodTicks;
    
    public HoodDistance(int inches, int hoodTicks){
        m_Inches = inches;
        m_HoodTicks = hoodTicks;
    }
    public int getInches(){
        return m_Inches;
    }
    public int getHoodTicks(){
        return m_HoodTicks;
    }
}
