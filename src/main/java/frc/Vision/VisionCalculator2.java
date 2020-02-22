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
public class VisionCalculator2 {
    
    private ArrayList<VisionDistance> positions;
    
    public void setupPositions() {
        positions = new ArrayList<VisionDistance> (); //in inches from goal
        //                               D        TY      TA     TS  THOR
        positions.add(new VisionDistance(3 * 12,  23.2f,  1.77f, 0f, 117f)); // 1
        positions.add(new VisionDistance(4 * 12,  18.9f,  7.36f, 0f, 146f)); // 2
        positions.add(new VisionDistance(5 * 12,  12.2f,  6.0f,  0f, 126f)); // 3
        positions.add(new VisionDistance(6 * 12,  6.9f,   5.2f,  0f, 112f)); // 4
        positions.add(new VisionDistance(7 * 12,  2.8f,   4.5f,  0f, 100f)); // 5
        positions.add(new VisionDistance(8 * 12,  -1.5f,  4.4f,  0f, 92f));  // 6
        positions.add(new VisionDistance(9 * 12,  -3.2f,  3.6f,  0f, 83f));  // 7
        positions.add(new VisionDistance(10 * 12, -5.5f,  3.2f,  0f, 78f));  // 8
        positions.add(new VisionDistance(11 * 12, -7.0f,  2.7f,  0f, 72f));  // 9
        positions.add(new VisionDistance(12 * 12, -8.6f,  2.2f,  0f, 66f));  // 10
        positions.add(new VisionDistance(13 * 12, -9.8f,  1.8f,  0f, 62f));  // 11
        positions.add(new VisionDistance(14 * 12, -11.0f, 1.4f,  0f, 56f));  // 12
        positions.add(new VisionDistance(15 * 12, -11.9f, 1.3f,  0f, 53f));  // 13
        positions.add(new VisionDistance(16 * 12, -12.8f, 1.1f,  0f, 50f));  // 14
        positions.add(new VisionDistance(17 * 12, -13.3f, 1.0f,  0f, 48f));  // 15
        positions.add(new VisionDistance(18 * 12, -14.2f, 0.9f,  0f, 46f));  // 16
        positions.add(new VisionDistance(19 * 12, -15.0f, 0.9f,  0f, 44f));  // 17
        positions.add(new VisionDistance(20 * 12, -15.5f, 0.7f,  0f, 42f));  // 18
        positions.add(new VisionDistance(21 * 12, -16.3f, 0.7f,  0f, 40f));  // 19
        positions.add(new VisionDistance(22 * 12, -16.9f, 0.6f,  0f, 38f));  // 20
        positions.add(new VisionDistance(23 * 12, -17.1f, 0.5f,  0f, 36f));  // 21
        positions.add(new VisionDistance(24 * 12, -17.7f, 0.5f,  0f, 35f));  // 22
        positions.add(new VisionDistance(25 * 12, -18.4f, 0.5f,  0f, 34f));  // 23
        positions.add(new VisionDistance(26 * 12, -18.6f, 0.4f,	0f,	32));   // 24
        positions.add(new VisionDistance(27 * 12, -18.7f, 0.4f,	0f,	32));   // 25
        positions.add(new VisionDistance(28 * 12, -19.9f, 0.4f,	0f,	31));   // 26
        positions.add(new VisionDistance(29 * 12, -19.4f, 0.4f,	0f,	30));   // 27
        positions.add(new VisionDistance(30 * 12, -19.8f, 0.3f,	0f,	28));   // 28 *
        positions.add(new VisionDistance(31 * 12, -19.9f, 0.3f,	0f,	27));   // 29
        positions.add(new VisionDistance(32 * 12, -20.2f, 0.3f,	0f,	28));   // 30
        positions.add(new VisionDistance(33 * 12, -20.5f, 0.3f,	0f,	27));   // 31
        positions.add(new VisionDistance(34 * 12, -21.0f, 0.3f,	0f,	26));   // 32
        positions.add(new VisionDistance(35 * 12, -21.1f, 0.3f,	0f,	24));   // 33
        positions.add(new VisionDistance(36 * 12, -21.3f, 0.3f,	0f,	24));   // 34
        positions.add(new VisionDistance(37 * 12, -21.5f, 0.3f,	0f,	24));   // 35
        positions.add(new VisionDistance(38 * 12, -21.6f, 0.3f,	0f,	23));   // 36
        positions.add(new VisionDistance(39 * 12, -21.7f, 0.3f,	0f,	24));   // 37
    }

    public float getDistance(float ty, float ta, float ts, float thor) {

        VisionDistance before = null;
        VisionDistance after = null;

        for(int i = 0; i < positions.size(); i++) {
            if(positions.get(i).getTY() < ty) {
                before = positions.get(i);
            } else {
                after = positions.get(i);
                break;
            }
        }

        if(before == null || after == null) {
            return positions.get(8).getDistance();
        }

        float pct = (ty - before.getTY() / 100f);
        return after.getDistance() + before.getDistance() * pct;
    }

}