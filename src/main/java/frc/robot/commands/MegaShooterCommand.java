/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
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

  private boolean manualShooterIncrease;
  private boolean manualShooterDecrease;
  private boolean manualHoodUp;
  private boolean manualHoodDown;
  private boolean manualTurretLeft;
  private boolean manualTurretRight;
  private boolean shootPressed;
  private boolean readyPressed;
  private boolean conveyorDownPressed;
  private boolean conveyorUpPressed;

  public MegaShooterCommand(ShooterSubsystem shooter, VisionSubsystem vision, HoodSubsystem hood, TurretSubsystem turret, ConveyorSubsystem conveyor) {
    m_shooter = shooter;
    m_vision = vision;
    m_hood = hood;
    m_turret = turret;
    m_conveyor = conveyor;

    addRequirements(shooter, vision, hood, turret, conveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  public void update(boolean shooterIncrease, boolean shooterDecrease, 
                      boolean hoodUp, boolean hoodDown, 
                      boolean turretLeft, boolean turretRight, 
                      boolean ready, boolean shoot, boolean conveyorDown, boolean conveyorUp) {

    manualShooterIncrease = shooterIncrease;
    manualShooterDecrease = shooterDecrease;
    manualHoodUp = hoodUp;
    manualHoodDown = hoodDown;
    manualTurretLeft = turretLeft;
    manualTurretRight = turretRight;
    readyPressed = ready;
    shootPressed = shoot;
    conveyorDownPressed = conveyorDown;  
  }

  public void executeHood() {
    m_vision.updateLimelight();

    if(SmartDashboard.getBoolean(Constants.SmartDashboard.kHoodOverrideString, false)) {
      hoodOnTarget = true;
      if(manualHoodUp) {
        m_hood.manualMoveHoodUp();
      } else if(manualHoodDown) {
        m_hood.manualMoveHoodDown();
      } else {
        m_hood.manualStopHoodMovement();
      }
    } else {
      double hoodTargetAngle = m_vision.getTy();
      //TODO: this is going to need more math
      //calculate the hood angle from the hood system
      double hoodCurrentAngle = m_hood.getCurrentAngle();
      hoodOnTarget = Math.abs(hoodTargetAngle - hoodCurrentAngle) < .5;
      //turn hood to target angle 
    }
  }

  public void executeTurret() {
    m_vision.updateLimelight();

    if(SmartDashboard.getBoolean(Constants.SmartDashboard.kTurretOverrideString, false)){
      turretOnTarget = true;
      if(manualTurretLeft) {
        m_turret.turnTurret(-0.1);
      } if(manualTurretRight) {
        m_turret.turnTurret(0.1);
      } else {
        m_turret.turnTurret(0);
      }
    } else if(m_vision.hasValidTarget()){
      double turretTargetAngle = m_vision.getTx(); //calculate target turret angle from vision
      turretOnTarget = m_turret.getIsOnTarget();
      m_turret.driveToPos(turretTargetAngle);//turn turret to target angle
    } else {
      m_turret.turnTurret(0);
    }
  }

  public void executeShooter() {
    if(SmartDashboard.getBoolean(Constants.SmartDashboard.kShooterOverrideString, false)) {
      speedOnTarget = true;
      double currentPowerPct = m_shooter.getSpeed();
      if(manualShooterIncrease) {
        currentPowerPct += 0.5;
        if(currentPowerPct > 1) {
          currentPowerPct = 1;
        }
        m_shooter.setSpeed(currentPowerPct);
      } else if (manualShooterDecrease) {
        currentPowerPct -= 0.5;
        if(currentPowerPct < 0) {
          currentPowerPct = 0;
        }
        m_shooter.setSpeed(currentPowerPct);
      } else {
        m_shooter.setSpeed(currentPowerPct);
      }
    } else {
      int targetSpeed = 0; //TODO: calculate targetSpeed in shooter 
      speedOnTarget = Math.abs(m_shooter.getSpeed() - targetSpeed) < .5;
      //m_shooter.setSpeed(targetSpeed);
    }
  }



  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(shootPressed || readyPressed) {
      m_vision.setLEDMode(0);
      executeHood();
      executeTurret();
      executeShooter();
      if(hoodOnTarget && turretOnTarget && speedOnTarget && shootPressed) {
        if(conveyorDownPressed == false) {
          m_conveyor.lifterUp();
        } else {
          m_conveyor.lifterDown();
        }
      } else if(conveyorUpPressed && SmartDashboard.getBoolean(Constants.SmartDashboard.kConveyorOverrideString, false)) {
          m_conveyor.lifterUp();
      } else {
        m_conveyor.lifterStop();
      }
    } else {
      m_vision.setLEDMode(1);
      m_shooter.setSpeed(0);
      m_turret.turnTurret(0);
    }
  }

  public void setShootPressed(boolean isShoot) {
    shootPressed = isShoot;
  }

  public boolean getIsReady() {
    return hoodOnTarget && turretOnTarget && speedOnTarget;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_hood.manualStopHoodMovement();
    m_conveyor.lifterStop();
    m_shooter.setSpeed(0);
    m_turret.turnTurret(0);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;  //never "done" because it is the default command
  }
}
