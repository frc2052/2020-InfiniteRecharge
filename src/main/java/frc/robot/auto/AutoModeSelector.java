/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.SmartDashboardStrings;

public class AutoModeSelector {

    private static SendableChooser<autos> sendableChooserAutos;

    public static void putToShuffleBoard() {
        sendableChooserAutos = new SendableChooser<autos>();
        
        for(int i = 0; i < autos.values().length; i++) {
            autos mode = autos.values()[i];
            if (i == 0) {
                sendableChooserAutos.setDefaultOption(mode.name, mode);
            } else {
                sendableChooserAutos.addOption(mode.name, mode);
            }
        }

        SmartDashboard.putData("Autos", sendableChooserAutos);
        SmartDashboard.putNumber(Constants.SmartDashboardStrings.kDistanceToLeftWallInches, 0);
        SmartDashboard.putNumber(Constants.SmartDashboardStrings.kDistanceToRightWallInches, 0);
        SmartDashboard.putNumber(Constants.SmartDashboardStrings.kHoodTrim, 0);
    }

    public static double getPosOnLineInches() {
        double leftInches = SmartDashboard.getNumber(Constants.SmartDashboardStrings.kDistanceToLeftWallInches, 0);
        double rightInches = SmartDashboard.getNumber(Constants.SmartDashboardStrings.kDistanceToRightWallInches, 0);

        double leftWallPosInches = -228; //TODO: find the real values for these, this isn't correct
        double rightWallPosInches = 96;

        double posOnLine;

        if(rightInches != 0) { // check right side first
            posOnLine = rightWallPosInches - rightInches; //right side is positive
        } else if(leftInches != 0) { 
            posOnLine = leftWallPosInches + leftInches; //left side is negative
        } else {
            posOnLine = 0; //if we don't measure from anywhere, default to being at 0
        }

        return posOnLine;
    }

    public static autos getSelectedAuto() {
        autos selectedAuto = sendableChooserAutos.getSelected();
        return selectedAuto;
    }

    public enum autos {
        DRIVE("drive"),
        DM("don't move"),
        CS("center start shoot drive park"),
        CSG2("center start shoot generator 2"),
        //LSG3("left start shoot generator 3"),
        LST2("left start shoot trench 2"),
        RST32("right start trench 3(bad balls)"),
        //LSG5("left start shoot generator 5"),
        RST3("right start shoot trench 3");

        public String name;

        autos(String name) {
            this.name = name;
        }
    }

    public enum posOnLine {
        MIDDLE("Middle"),
        FORWARD("Forward"),
        BACK("Back");

        public String name;
        
        posOnLine(String name) {
            this.name = name;
        }
    }

    public enum directionMeasured {
        LEFT("Left"),
        RIGHT("Right");

        public String name;

        directionMeasured(String name) {
            this.name = name;
        }
    }
}
