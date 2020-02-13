/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class VisionSubsystem extends SubsystemBase {
  
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  private NetworkTableEntry tvI = table.getEntry("tv"); // whether there are any targets
  private NetworkTableEntry txI = table.getEntry("tx"); // target x
  private NetworkTableEntry tyI = table.getEntry("ty"); // target y
  private NetworkTableEntry taI = table.getEntry("ta"); // can be used for distance sensing in the future

  private NetworkTableEntry pipelineI = table.getEntry("getpipe"); // pipeline
  private NetworkTableEntry ledModeI = table.getEntry("ledMode"); // led mode
  private NetworkTableEntry camModeI = table.getEntry("camMode"); // cam mode
  
  private NetworkTableEntry thorI = table.getEntry("thor"); // bounding box horizontal
  private NetworkTableEntry tvertI = table.getEntry("tvert"); // bounding box vertical

  private NetworkTableEntry tshortI = table.getEntry("tshort"); // target shortest side
  private NetworkTableEntry tlongI = table.getEntry("tlong"); // target longest side

  private double tv; // whether there are any targets
  private double tx; // target x
  private double ty; // target y
  private double ta; // can be used for distance sensing in the future (target area)

  private double pl; // pipeline
  private double lm; // led mode
  private double cm; // cam mode

  private double thor; // bounding box horizontal
  private double tvert; // bounding box vertical

  private double tshort; // target shortest side
  private double tlong; // target longest side

  private boolean hasValidTarget; // whether or not there are any targets visible, < 1 = no target, 1 = target

  public void updateLimelight() {

    this.tv = this.tvI.getDouble(0.0);
    this.tx = this.txI.getDouble(0.0);
    this.ty = this.tyI.getDouble(0.0);
    this.ta = this.taI.getDouble(0.0);

    this.pl = this.pipelineI.getDouble(0.0);
    this.lm = this.ledModeI.getDouble(0.0);
    this.cm = this.camModeI.getDouble(0.0);

    this.thor = this.thorI.getDouble(0.0);
    this.tvert = this.tvertI.getDouble(0.0);

    this.tshort = this.tshortI.getDouble(0.0);
    this.tlong = this.tlongI.getDouble(0.0);

    //System.out.println("contour box horizontal: " + this.getContourHorizontal());
    //System.out.println("contour box vertical: " + this.getContourVertical());

    if(!hasValidTarget)
      return; // search for target in the future?

  }

  public double getTv() {return this.tv;}
  public double getTx() {return this.tx;}
  public double getTy() {return ty;}
  public double getTa() {return this.ta;}

  public int getPl() {return (int)this.pl;}
  public int getLm() {return (int)this.lm;}
  public int getCm() {return (int)this.cm;}

  public double getThor() {return this.thor;}
  public double getTvert() {return this.tvert;}

  public double getTshort() {return this.tshort;}
  public double getTlong() {return this.tlong;}

  public boolean hasValidTarget() {
    if (this.tv < 1.0) 
      return false; 
    else 
      return true;
  }

  public void setLEDMode(int ledMode) {
    if(ledMode >= 0 && ledMode <= 3) // 4 led modes
      ledModeI.setDouble((double) ledMode);
    /*
    0 = use default LED mode in pipeline
    1 = force off
    2 = force blink
    3 = force on
    */
  }

  public void setCamMode(int camMode) {
    if(camMode == 0 || camMode == 1)
      camModeI.setDouble((double) camMode);
    /*
    0 = vision processor
    1 = driver camera (increases exposure, disables vision processing)
    */
  }

  public void setPipeline(int pipeline) {
    if(pipeline >= 0 && pipeline <= 9) // 10 pipelines
      pipelineI.setDouble((double) pipeline);     
  }

  public double getDistanceToTargetInches() { // find distance based on ta and distance ratio INCHES
    updateLimelight();
    return (Constants.Vision.kTargetHeight - Constants.Vision.kCameraMountingHeight) / (Math.tan(Math.toRadians(Constants.Vision.kCameraMountingAngleY + this.getTy())));
  }

  public void putDistanceToSmartDashboard() {
    SmartDashboard.putNumber("Distance from target", getDistanceToTargetInches());
  }
  
}
