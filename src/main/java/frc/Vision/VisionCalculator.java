/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.Vision;

import java.util.ArrayList;

/**
 * Add your docs here.
 */
public class VisionCalculator {
    private ArrayList<HoodDistance> hoodTicks = new ArrayList<HoodDistance> ();//to score a goal
    
    public VisionCalculator(){
     SetupHoodCalcs();
    }


    public void SetupHoodCalcs()
    {
        hoodTicks.add(new HoodDistance(36, 557));
        hoodTicks.add(new HoodDistance(48,611));
        hoodTicks.add(new HoodDistance(60, 688));
        hoodTicks.add(new HoodDistance(72, 911));
        hoodTicks.add(new HoodDistance(84, 1032));
        hoodTicks.add(new HoodDistance(96, 1108));
        hoodTicks.add(new HoodDistance(108, 1147));
        hoodTicks.add(new HoodDistance(120, 1180));
        hoodTicks.add(new HoodDistance(132, 1227));
        hoodTicks.add(new HoodDistance(144, 1232));
        hoodTicks.add(new HoodDistance(156, 1259));
        hoodTicks.add(new HoodDistance(168, 1268));
        hoodTicks.add(new HoodDistance(180, 1256));
        hoodTicks.add(new HoodDistance(192, 1262));
        hoodTicks.add(new HoodDistance(204, 1261));
        hoodTicks.add(new HoodDistance(216, 1261));
        hoodTicks.add(new HoodDistance(228, 1254));
        hoodTicks.add(new HoodDistance(240, 1253));
        hoodTicks.add(new HoodDistance(252, 1253));
        hoodTicks.add(new HoodDistance(264, 1253));
        hoodTicks.add(new HoodDistance(276, 1240));
        hoodTicks.add(new HoodDistance(288, 1221));
        hoodTicks.add(new HoodDistance(300, 1190));
        hoodTicks.add(new HoodDistance(312, 1193));
        hoodTicks.add(new HoodDistance(324, 1193));
        hoodTicks.add(new HoodDistance(336, 1193));
        hoodTicks.add(new HoodDistance(348, 1178));
        hoodTicks.add(new HoodDistance(360, 1150));
        hoodTicks.add(new HoodDistance(372, 1128));
    }
    public int DistanceToTicks(int inches){
        int index = 0;
        while(index < (hoodTicks.size() - 1) && inches < hoodTicks.get(index).getInches()){
            index++;
        }
        index--;
        
        HoodDistance before = null;
        HoodDistance after = null;

        for(int i = 0; i < hoodTicks.size(); i++){
            if (hoodTicks.get(i).getInches() < inches){
                before = hoodTicks.get(i);
            } else {
                after = hoodTicks.get(i);
                break;
            }
        }
        
        if(before == null || after == null){
            return hoodTicks.get(7).getHoodTicks();
        }
        return (before.getHoodTicks() + ((after.getHoodTicks()- before.getHoodTicks()) * ((after.getInches() - inches)-12)));
    }

}