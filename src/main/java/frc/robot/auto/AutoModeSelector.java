/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoModeSelector {
    //TODO: finish this class once we have auto modes
    //TODO: make this more complex for easier auto mode selection

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

        SmartDashboard.putData("Possible Autos", sendableChooserAutos);
    }

    public static autos getSelectedAuto() {
        autos selectedAuto = sendableChooserAutos.getSelected();
        return selectedAuto;
    }

    public enum autos {
        LSGENERATOR3("left start shoot generator 3"),
        LSTRENCH2("left start shoot trench 2"),
        LSSHOOT5("left start shoot 5"),
        CSGENERATOR3("center start shoot generator 3"),
        CSP("center start shoot drive park"),
        RSTRENCH3("right start shoot trench 3");

        public String name;

        autos(String name) {
            this.name = name;
        }
    }

}
