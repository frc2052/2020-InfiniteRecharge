/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MegaShooterCommand extends CommandBase {
  private ShooterSubsystem m_shooter;
  private VisionSubsystem m_vision;
  private HoodSubsystem m_hood;
  private TurretSubsystem m_turret;
  private ConveyorSubsystem m_conveyor;

  private boolean hoodOnTarget = false;
  private boolean turretOnTarget = false;
  private boolean speedOnTarget = false;

  private boolean hoodUp;

  public MegaShooterCommand(ShooterSubsystem shooter, VisionSubsystem vision, HoodSubsystem hood, TurretSubsystem turret, ConveyorSubsystem conveyor,
                              boolean manualShooterIncrease, boolean manualShooterDecrease, 
                              boolean manualHoodUp, boolean manualHoodDown,
                              boolean manualTurretLeft, boolean manualTurretRight,
                              boolean shootPressed, boolean readyPressed) {
    m_shooter = shooter;
    m_vision = vision;
    m_hood = hood;
    m_turret = turret;
    m_conveyor = conveyor;

    hoodUp = manualHoodUp;

    addRequirements(shooter, vision, hood, turret, conveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  public void executeHood() {
    if(SmartDashboard.getBoolean("Hood Override?", false)) {
      hoodOnTarget = true;
      if(hoodUp) {
        m_hood.startEmergencyUp();
      } else if(manualHoodDown) {
        m_hood.startEmergencyDown();
      } else {
        m_hood.stopEmergencyMove();
      }
    } else {
      double hoodTargetAngle = 0;
      //calculate the hood angle from the vision system
      //set hood angle 
      double hoodCurrentAngle = 0; //get the current able
      hoodOnTarget = Math.abs(hoodTargetAngle - hoodCurrentAngle) < .5;
    }
  }

  public void executeTurret() {
    if(SmartDashboard.getBoolean("Turret Override?", false)){
      turretOnTarget = true;
      if(manualTurretLeft) {
        m_turret.turnTurret(-0.1);
      } if(manualTurretRight) {
        m_turret.turnTurret(0.1);
      } else {
        m_turret.turnTurret(0);
      }
    } else {
      double turretCurrentAngle = 0; //get current turret angle from turret
      double turretTargetAngle = 0; //calculate target turret angle from vision
    }
  }

  public void executeShooter() {
    if(SmartDashboard.getBoolean("Shooter Override?", false)) {
      speedOnTarget = true;
      currentPowerPct = m_shooter.getSpeed();
    }
  }



  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(shootPressed || readyPressed) {
      executeHood();
      executeTurret();
      if(hoodOnTarget && turretOnTarget && speedOnTarget && shootPressed) {
        //run conveyor
      } else {
        //stop conveyor
      }
    } else {
      //TODO: Stop all subsystems
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
