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

  private boolean manualShooterIncrease;
  private boolean manualShooterDecrease;
  private boolean manualHoodUp;
  private boolean manualHoodDown;
  private boolean manualTurretLeft;
  private boolean manualTurretRight;
  private boolean shootPressed;
  private boolean readyPressed;
  private boolean conveyorDownPressed;

  public MegaShooterCommand(ShooterSubsystem shooter, VisionSubsystem vision, HoodSubsystem hood, TurretSubsystem turret, ConveyorSubsystem conveyor,
                              boolean shooterIncrease, boolean shooterDecrease, 
                              boolean hoodUp, boolean hoodDown,
                              boolean turretLeft, boolean turretRight,
                              boolean shoot, boolean ready,
                              boolean conveyorDown) {
    m_shooter = shooter;
    m_vision = vision;
    m_hood = hood;
    m_turret = turret;
    m_conveyor = conveyor;

    manualHoodUp = hoodUp;
    manualHoodDown = hoodDown;
    manualTurretLeft = turretLeft;
    manualTurretRight = turretRight;
    shootPressed = shoot;
    readyPressed = ready;
    conveyorDownPressed = conveyorDown;

    addRequirements(shooter, vision, hood, turret, conveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  public void executeHood() {
    if(SmartDashboard.getBoolean("Hood Override?", false)) {
      hoodOnTarget = true;
      if(manualHoodUp) {
        m_hood.startEmergencyUp();
        manualHoodUp = false;
      } else if(manualHoodDown) {
        m_hood.startEmergencyDown();
        manualHoodDown = false;
      } else {
        m_hood.stopEmergencyMove();
      }
    } else {
      double hoodTargetAngle = m_vision.getTy();
      //calculate the hood angle from the hood system
      double hoodCurrentAngle = 0; //get the current angle
      hoodOnTarget = Math.abs(hoodTargetAngle - hoodCurrentAngle) < .5;
      //turn turret to target angle 
    }
  }

  public void executeTurret() {
    if(SmartDashboard.getBoolean("Turret Override?", false)){
      turretOnTarget = true;
      if(manualTurretLeft) {
        m_turret.turnTurret(-0.1);
        manualTurretLeft = false;
      } if(manualTurretRight) {
        m_turret.turnTurret(0.1);
        manualTurretRight = false;
      } else {
        m_turret.turnTurret(0);
      }
    } else {
      double turretCurrentAngle = m_turret.getTurretDegree(); //get current turret angle from turret
      double turretTargetAngle = m_vision.getTx(); //calculate target turret angle from vision
      turretOnTarget = m_turret.getIsOnTarget();
      m_turret.driveToPos(turretTargetAngle);//turn turret to target angle using motion magic
    }
  }

  public void executeShooter() {
    if(SmartDashboard.getBoolean("Shooter Override?", false)) {
      speedOnTarget = true;
      double currentPowerPct = m_shooter.getSpeed();
      if(manualShooterIncrease) {
        currentPowerPct += 0.5;
        if(currentPowerPct > 1) {
          currentPowerPct = 1;
        }
        m_shooter.setSpeed(currentPowerPct);
        manualShooterIncrease = false;
      } else if (manualShooterDecrease) {
        currentPowerPct -= 0.5;
        if(currentPowerPct < 0) {
          currentPowerPct = 0;
        }
        m_shooter.setSpeed(currentPowerPct);
        manualShooterDecrease = false; 
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
      executeHood();
      executeTurret();
      executeShooter();
      if(hoodOnTarget && turretOnTarget && speedOnTarget && shootPressed) {
        if(conveyorDownPressed == false) {
          m_conveyor.lifterUp();
        } else {
          m_conveyor.lifterDown();
        }
      } else {
        m_conveyor.lifterStop();
      }
    } else {
      m_shooter.setSpeed(0);
      m_turret.turnTurret(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_turret.setIsOnTarget(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
