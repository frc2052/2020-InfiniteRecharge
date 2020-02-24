/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.vision;

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
        hoodTicks.add(new HoodDistance(36, 557, 1));
        hoodTicks.add(new HoodDistance(48,611, 1));
        hoodTicks.add(new HoodDistance(60, 688, 1));
        hoodTicks.add(new HoodDistance(72, 911, 1));
        hoodTicks.add(new HoodDistance(84, 1032, 1));
        hoodTicks.add(new HoodDistance(96, 1108, 1));
        hoodTicks.add(new HoodDistance(108, 1147, 1));
        hoodTicks.add(new HoodDistance(120, 1180, 1));
        hoodTicks.add(new HoodDistance(132, 1227, 1));
        hoodTicks.add(new HoodDistance(144, 1232, 1));
        hoodTicks.add(new HoodDistance(156, 1259, 1));
        hoodTicks.add(new HoodDistance(168, 1268, 1));
        hoodTicks.add(new HoodDistance(180, 1256, 1));
        hoodTicks.add(new HoodDistance(192, 1262, 1));
        hoodTicks.add(new HoodDistance(204, 1261, 1));
        hoodTicks.add(new HoodDistance(216, 1261, 1));
        hoodTicks.add(new HoodDistance(228, 1254, 1));
        hoodTicks.add(new HoodDistance(240, 1253, 1));
        hoodTicks.add(new HoodDistance(252, 1253, 1));
        hoodTicks.add(new HoodDistance(264, 1253, 1));
        hoodTicks.add(new HoodDistance(276, 1240, 1));
        hoodTicks.add(new HoodDistance(288, 1221, 1));
        hoodTicks.add(new HoodDistance(300, 1190, 1));
        hoodTicks.add(new HoodDistance(312, 1193, 1));
        hoodTicks.add(new HoodDistance(324, 1193, 1));
        hoodTicks.add(new HoodDistance(336, 1193, 1));
        hoodTicks.add(new HoodDistance(348, 1178, 1));
        hoodTicks.add(new HoodDistance(360, 1150, 1));
        hoodTicks.add(new HoodDistance(372, 1128, 1));
    }

    public void setupPositions() {
        //                                      D        TY      TA     TS  THOR
        visionPositions.add(new VisionDistance(3 * 12,  23.2f,  1.77f, 0f, 117f)); // 1
        visionPositions.add(new VisionDistance(4 * 12,  18.9f,  7.36f, 0f, 146f)); // 2
        visionPositions.add(new VisionDistance(5 * 12,  12.2f,  6.0f,  0f, 126f)); // 3
        visionPositions.add(new VisionDistance(6 * 12,  6.9f,   5.2f,  0f, 112f)); // 4
        visionPositions.add(new VisionDistance(7 * 12,  2.8f,   4.5f,  0f, 100f)); // 5
        visionPositions.add(new VisionDistance(8 * 12,  -1.5f,  4.4f,  0f, 92f));  // 6
        visionPositions.add(new VisionDistance(9 * 12,  -3.2f,  3.6f,  0f, 83f));  // 7
        visionPositions.add(new VisionDistance(10 * 12, -5.5f,  3.2f,  0f, 78f));  // 8
        visionPositions.add(new VisionDistance(11 * 12, -7.0f,  2.7f,  0f, 72f));  // 9
        visionPositions.add(new VisionDistance(12 * 12, -8.6f,  2.2f,  0f, 66f));  // 10
        visionPositions.add(new VisionDistance(13 * 12, -9.8f,  1.8f,  0f, 62f));  // 11
        visionPositions.add(new VisionDistance(14 * 12, -11.0f, 1.4f,  0f, 56f));  // 12
        visionPositions.add(new VisionDistance(15 * 12, -11.9f, 1.3f,  0f, 53f));  // 13
        visionPositions.add(new VisionDistance(16 * 12, -12.8f, 1.1f,  0f, 50f));  // 14
        visionPositions.add(new VisionDistance(17 * 12, -13.3f, 1.0f,  0f, 48f));  // 15
        visionPositions.add(new VisionDistance(18 * 12, -14.2f, 0.9f,  0f, 46f));  // 16
        visionPositions.add(new VisionDistance(19 * 12, -15.0f, 0.9f,  0f, 44f));  // 17
        visionPositions.add(new VisionDistance(20 * 12, -15.5f, 0.7f,  0f, 42f));  // 18
        visionPositions.add(new VisionDistance(21 * 12, -16.3f, 0.7f,  0f, 40f));  // 19
        visionPositions.add(new VisionDistance(22 * 12, -16.9f, 0.6f,  0f, 38f));  // 20
        visionPositions.add(new VisionDistance(23 * 12, -17.1f, 0.5f,  0f, 36f));  // 21
        visionPositions.add(new VisionDistance(24 * 12, -17.7f, 0.5f,  0f, 35f));  // 22
        visionPositions.add(new VisionDistance(25 * 12, -18.4f, 0.5f,  0f, 34f));  // 23
        visionPositions.add(new VisionDistance(26 * 12, -18.6f, 0.4f,	0f,	32));   // 24
        visionPositions.add(new VisionDistance(27 * 12, -18.7f, 0.4f,	0f,	32));   // 25
        visionPositions.add(new VisionDistance(28 * 12, -19.9f, 0.4f,	0f,	31));   // 26
        visionPositions.add(new VisionDistance(29 * 12, -19.4f, 0.4f,	0f,	30));   // 27
        visionPositions.add(new VisionDistance(30 * 12, -19.8f, 0.3f,	0f,	28));   // 28 *
        visionPositions.add(new VisionDistance(31 * 12, -19.9f, 0.3f,	0f,	27));   // 29
        visionPositions.add(new VisionDistance(32 * 12, -20.2f, 0.3f,	0f,	28));   // 30
        visionPositions.add(new VisionDistance(33 * 12, -20.5f, 0.3f,	0f,	27));   // 31
        visionPositions.add(new VisionDistance(34 * 12, -21.0f, 0.3f,	0f,	26));   // 32
        visionPositions.add(new VisionDistance(35 * 12, -21.1f, 0.3f,	0f,	24));   // 33
        visionPositions.add(new VisionDistance(36 * 12, -21.3f, 0.3f,	0f,	24));   // 34
        visionPositions.add(new VisionDistance(37 * 12, -21.5f, 0.3f,	0f,	24));   // 35
        visionPositions.add(new VisionDistance(38 * 12, -21.6f, 0.3f,	0f,	23));   // 36
        visionPositions.add(new VisionDistance(39 * 12, -21.7f, 0.3f,	0f,	24));   // 37
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

        if(resultingInches < after.getDistance() && resultingInches > before.getDistance()) {
            System.out.println("OFFSET INCHES====" + resultingInches + "    BEFORE INCHES ===" + before.getDistance() + "    AFTER INCHES===" + after.getDistance());
        } else {
            System.out.println("CALCULATED HOOD INCHES WRONG===" + offsetInches +  "    BEFORE INCHES ===" + before.getDistance() + "    AFTER INCHES===" + after.getDistance());
        }

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

        if(resultingTicks < after.getHoodTicks() && resultingTicks > before.getHoodTicks()) {
            System.out.println("OFFSET TICKS====" + resultingTicks + "    BEFORE TICKS ===" + before.getHoodTicks() + "    AFTER TICKS===" + after.getHoodTicks());
        } else {
            System.out.println("CALCULATED HOOD TICKS WRONG===" + resultingTicks +  "    BEFORE TICKS ===" + before.getHoodTicks() + "    AFTER TICKS===" + after.getHoodTicks());
        }

        //return (int)(before.getHoodTicks() + ((after.getHoodTicks() - before.getHoodTicks()) * pct));
        return (int) (before.getHoodTicks() + offsetTicks);
    }

}