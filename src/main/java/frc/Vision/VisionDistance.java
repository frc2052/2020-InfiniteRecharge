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
public class VisionDistance {

    private int m_distance;
    private float m_ty;
    private float m_ta;
    private float m_ts;
    private float m_thor;

    public VisionDistance(int distance, float ty, float ta, float ts, float thor) {
        this.m_distance = distance;
        this.m_ty = ty;
        this.m_ta = ta;
        this.m_ts = ts;
        this.m_thor = thor;
    }

    public int getDistance() {
        return this.m_distance;
    }

    public float getTY() {
        return this.m_ty;
    }

    public float getTA() {
        return this.m_ta;
    }

    public float getTS() {
        return this.m_ts;
    }

    public float getTHOR() {
        return this.m_thor;
    }

}
