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

    private final double shooterMotorSpeedAdjustment = 1.0;

    private final double shooterMotorSpeedPerInch = 0.1;

    //this is based on a top down x & y cord system
    public double getDistanceToTargetInches(double ty) { // find distance based on ta and distance ratio INCHES
        return ((Constants.Vision.kCameraMountingHeight) / (Math.tan(Constants.Vision.kCameraMountingAngleY + ty)));
    }

    public double getShooterMotorSpeed() {
        double distance = this.getDistanceToTargetInches(camSubsystem.getTy());
        return (distance * shooterMotorSpeedPerInch) + shooterMotorSpeedAdjustment;
    }

}