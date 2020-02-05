/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class VisionTurretAdjustCommand extends CommandBase {
  private VisionSubsystem vision;
  private TurretSubsystem turret;
  private boolean m_IsFinished = false;


  public VisionTurretAdjustCommand(VisionSubsystem visionSubsystem, TurretSubsystem turretSubsystem) {
    vision = visionSubsystem;
    turret = turretSubsystem;

    addRequirements(vision, turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_IsFinished = false;
    vision.setLEDMode(3);
    System.out.println("*******************************Vision Command Starting**************************************");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    vision.updateLimelight();
    if(vision.hasValidTarget()) {
      double turretTargetXAngle = vision.getTx(); //calculate target turret angle from vision

      
      //turretOnTarget = Math.abs(turretCurrentAngle - turretTargetAngle) < .5;
      turret.driveToPos(turretTargetXAngle);//turn turret to target angle using motion magic
      //System.out.println(turretTargetAngle);
      System.out.println(turretTargetXAngle);
    } else {
      turret.turnTurret(0);
      System.out.println("-----------------------No target");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IsFinished = true;
    turret.turnTurret(0);
    vision.setLEDMode(1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_IsFinished;
  }
}
