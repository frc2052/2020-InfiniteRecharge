/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.vision;

import java.util.ArrayList;

/**
 * Add your docs here.
 */
public class VisionCalculator {
    private ArrayList<HoodDistance> hoodTicks = new ArrayList<HoodDistance> ();//to score a goal
    private ArrayList<VisionDistance> visionPositions = new ArrayList<VisionDistance> (); //in inches from goal
    
    public VisionCalculator(){
     setupHoodCalcs();
     setupPositions();
    }
    
    public void setupHoodCalcs()
    {
        //hoodTicks.add(new HoodDistance(0, 485, 1));
        hoodTicks.add(new HoodDistance(12*3, 485, 1));
        hoodTicks.add(new HoodDistance(12*4, 593, 1));
        hoodTicks.add(new HoodDistance(12*5, 666, 1));
        hoodTicks.add(new HoodDistance(12*6, 768, 1));
        hoodTicks.add(new HoodDistance(12*7, 830, 1));
        hoodTicks.add(new HoodDistance(12*8, 980, 1));
        hoodTicks.add(new HoodDistance(12*9, 1098, 1));
        hoodTicks.add(new HoodDistance(12*10, 1098, 1));
        hoodTicks.add(new HoodDistance(12*11, 1108, 1));
        hoodTicks.add(new HoodDistance(12*12, 1139, 1));
        hoodTicks.add(new HoodDistance(12*13, 1149, 1));
        hoodTicks.add(new HoodDistance(12*14, 1188, 1));
        hoodTicks.add(new HoodDistance(12*15, 1188, 1));
        hoodTicks.add(new HoodDistance(12*16, 1188, 1));
        hoodTicks.add(new HoodDistance(12*17, 1220, 1));
        hoodTicks.add(new HoodDistance(12*18, 1224, 1));
        hoodTicks.add(new HoodDistance(12*19, 1223, 1));
        hoodTicks.add(new HoodDistance(12*20, 1219, 1));
        hoodTicks.add(new HoodDistance(12*21, 1226, 1));
        hoodTicks.add(new HoodDistance(12*22, 1230, 1));
        hoodTicks.add(new HoodDistance(12*23, 1209, 1));
        hoodTicks.add(new HoodDistance(12*24, 1204, 1));
        hoodTicks.add(new HoodDistance(12*25, 1174, 1));
        hoodTicks.add(new HoodDistance(12*26, 1160, 1));
        hoodTicks.add(new HoodDistance(12*27, 1173, 1));
        hoodTicks.add(new HoodDistance(12*28, 1177, 1));
        hoodTicks.add(new HoodDistance(12*29, 1143, 1));
        hoodTicks.add(new HoodDistance(12*30, 1080, 1));
        hoodTicks.add(new HoodDistance(12*31, 1011, 1));
        hoodTicks.add(new HoodDistance(12*32, 932,1));
        hoodTicks.add(new HoodDistance(12*33, 930,1));
        hoodTicks.add(new HoodDistance(12*34, 904,1));
        hoodTicks.add(new HoodDistance(12*35, 896,1));
        hoodTicks.add(new HoodDistance(12*36, 816, 1));
    }

    public void setupPositions() {
        //                                      D        TY      TA     TS  THOR
        visionPositions.add(new VisionDistance(3 * 12,  21.07f,  1.77f, 0f, 136f)); // 1
        visionPositions.add(new VisionDistance(4 * 12,  16.36f,  7.36f, 0f, 140f)); // 2
        visionPositions.add(new VisionDistance(5 * 12,  10.44f,  6.0f,  0f, 122f)); // 3
        visionPositions.add(new VisionDistance(6 * 12,  6.05f,   5.2f,  0f, 110f)); // 4
        visionPositions.add(new VisionDistance(7 * 12,  2.24f,   4.5f,  0f, 98f)); // 5
        visionPositions.add(new VisionDistance(8 * 12,  -1.12f,  4.4f,  0f, 90f));  // 6
        visionPositions.add(new VisionDistance(9 * 12,  -3.7f,  3.6f,  0f, 82f));  // 7
        visionPositions.add(new VisionDistance(10 * 12, -5.63f,  3.2f,  0f, 76f));  // 8
        visionPositions.add(new VisionDistance(11 * 12, -7.51f,  2.7f,  0f, 70f));  // 9
        visionPositions.add(new VisionDistance(12 * 12, -9.02f,  2.2f,  0f, 66f));  // 10
        visionPositions.add(new VisionDistance(13 * 12, -10.23f,  1.8f,  0f, 62f));  // 11
        visionPositions.add(new VisionDistance(14 * 12, -11.52f, 1.4f,  0f, 60f));  // 12
        visionPositions.add(new VisionDistance(15 * 12, -12.14f, 1.3f,  0f, 56f));  // 13
        visionPositions.add(new VisionDistance(16 * 12, -13.04f, 1.1f,  0f, 52f));  // 14
        visionPositions.add(new VisionDistance(17 * 12, -14.08f, 1.0f,  0f, 49f));  // 15
        visionPositions.add(new VisionDistance(18 * 12, -14.9f, 0.9f,  0f, 48f));  // 16
        visionPositions.add(new VisionDistance(19 * 12, -15.47f, 0.9f,  0f, 46f));  // 17
        visionPositions.add(new VisionDistance(20 * 12, -15.93f, 0.7f,  0f, 44f));  // 18
        visionPositions.add(new VisionDistance(21 * 12, -16.82f, 0.7f,  0f, 42f));  // 19
        visionPositions.add(new VisionDistance(22 * 12, -17.05f, 0.6f,  0f, 40f));  // 20
        visionPositions.add(new VisionDistance(23 * 12, -17.43f, 0.5f,  0f, 38f));  // 21
        visionPositions.add(new VisionDistance(24 * 12, -18.161f, 0.5f,  0f, 37f));  // 22
        visionPositions.add(new VisionDistance(25 * 12, -18.43f, 0.5f,  0f, 36f));  // 23
        visionPositions.add(new VisionDistance(26 * 12, -18.75f, 0.4f,	0f,	34));   // 24
        visionPositions.add(new VisionDistance(27 * 12, -19.15f, 0.4f,	0f,	32));   // 25
        visionPositions.add(new VisionDistance(28 * 12, -19.54f, 0.4f,	0f,	32));   // 26
        visionPositions.add(new VisionDistance(29 * 12, -19.74f, 0.4f,	0f,	30));   // 27
        visionPositions.add(new VisionDistance(30 * 12, -19.89f, 0.3f,	0f,	32));   // 28 *
        visionPositions.add(new VisionDistance(31 * 12, -20.46f, 0.3f,	0f,	30));   // 29
        visionPositions.add(new VisionDistance(32 * 12, -20.72f, 0.3f,	0f,	30));   // 30
        visionPositions.add(new VisionDistance(33 * 12, -21.2f, 0.3f,	0f,	28));   // 31
        visionPositions.add(new VisionDistance(34 * 12, -21.07f, 0.3f,	0f,	26));   // 32
        visionPositions.add(new VisionDistance(35 * 12, -21.14f, 0.3f,	0f,	26));  // 33
        visionPositions.add(new VisionDistance(36 * 12, -21.29f, 0.3f,	0f,	24));   // 34
        visionPositions.add(new VisionDistance(37 * 12, -21.5f, 0.3f,	0f,	24));   // 35
        visionPositions.add(new VisionDistance(38 * 12, -21.6f, 0.3f,	0f,	23));   // 36
        visionPositions.add(new VisionDistance(39 * 12, -21.9f, 0.3f,	0f,	24));   // 37
    }

    public int getDistance(double ty, double ta, double ts, double thor) {

        VisionDistance before = null;
        VisionDistance after = null;

        for(int i = 0; i < visionPositions.size(); i++) {
            if(visionPositions.get(i).getTY() > ty) {
                before = visionPositions.get(i);
            } else {
                after = visionPositions.get(i);
                break;
            }
        }

        if(before == null || after == null) {
            return visionPositions.get(8).getDistance(); //default to the initiation line position
        }

        double deltaY = before.getTY() - after.getTY();
        double offsetY = ty - after.getTY();


        double pct = offsetY/deltaY;
        double deltaInches = after.getDistance() - before.getDistance();
        double offsetInches = deltaInches * pct;

        int resultingInches = (int)(before.getDistance() + offsetInches);

        // if(resultingInches < after.getDistance() && resultingInches > before.getDistance()) {
        //     System.out.println("OFFSET INCHES====" + resultingInches + "    BEFORE INCHES ===" + before.getDistance() + "    AFTER INCHES===" + after.getDistance());
        // } else {
        //     System.out.println("CALCULATED HOOD INCHES WRONG===" + offsetInches +  "    BEFORE INCHES ===" + before.getDistance() + "    AFTER INCHES===" + after.getDistance());
        // }

//        return after.getDistance() + before.getDistance() * pct;
        return resultingInches;
    }

    public int distanceToTicks(int inches){
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
            return hoodTicks.get(7).getHoodTicks(); //by default, return the auto line distance
        }

        double deltaInches = before.getInches() - after.getInches();
        double offsetInches = inches - after.getInches();

        double pct = offsetInches / deltaInches;
        double deltaTicks = after.getHoodTicks() - before.getHoodTicks();
        double offsetTicks = deltaTicks * pct;

        int resultingTicks = (int)(offsetTicks + before.getHoodTicks());

        // if(resultingTicks < after.getHoodTicks() && resultingTicks > before.getHoodTicks()) {
        //     System.out.println("OFFSET TICKS====" + resultingTicks + "    BEFORE TICKS ===" + before.getHoodTicks() + "    AFTER TICKS===" + after.getHoodTicks());
        // } else {
        //     System.out.println("CALCULATED HOOD TICKS WRONG===" + resultingTicks +  "    BEFORE TICKS ===" + before.getHoodTicks() + "    AFTER TICKS===" + after.getHoodTicks());
        // }

        //return (int)(before.getHoodTicks() + ((after.getHoodTicks() - before.getHoodTicks()) * pct));
        return (int) (before.getHoodTicks() + offsetTicks);
    }

}