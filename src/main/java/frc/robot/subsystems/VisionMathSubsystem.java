/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Constants;

/**
 * http://docs.limelightvision.io/en/latest/cs_estimating_distance.html#using-a-fixed-angle-camera
 */

public class VisionMathSubsystem {

    private VisionSubsystem camSubsystem = new VisionSubsystem();

    //this is based on a top down x & y cord system
    public double getDistanceToTargetInches(double ty) { // find distance based on ta and distance ratio INCHES
        return ((Constants.Vision.kTargetHeight - Constants.Vision.kCameraMountingHeight) / (Math.tan(Constants.Vision.kCameraMountingAngleY + ty)));
    }

    // hood y angle
    public double getHoodAngleY() {
        return 0.0;
    }

    // turret x angle
    public double getTurretAngleX(tx) {
        return tx;
    }

}