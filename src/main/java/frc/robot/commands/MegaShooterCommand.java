/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.IShooterControls;
import frc.robot.ShooterControls;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MegaShooterCommand extends CommandBase {
  private ShooterSubsystem m_shooter;
  private VisionSubsystem m_vision;
  private HoodSubsystem m_hood;
  private TurretSubsystem m_turret;
  private ConveyorSubsystem m_conveyor;

  private IShooterControls shooterControls;

  private boolean hoodOnTarget = false;
  private boolean turretOnTarget = false;
  private boolean speedOnTarget = false;

  public MegaShooterCommand(ShooterSubsystem shooter, VisionSubsystem vision, HoodSubsystem hood, TurretSubsystem turret, ConveyorSubsystem conveyor, IShooterControls controls) {
    m_shooter = shooter;
    m_vision = vision;
    m_hood = hood;
    m_turret = turret;
    m_conveyor = conveyor;

    shooterControls = controls;

    addRequirements(shooter, hood, turret, conveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("*************** MEGA SHOOTER INIT");
  }

  public void executeHood() {
    if(SmartDashboard.getBoolean(Constants.SmartDashboard.kHoodOverrideString, false)) {
      hoodOnTarget = true;
      if(shooterControls.getManualHoodUp()) {
        System.out.println("HOOD UP");
        m_hood.manualMoveHoodUp();
      } else if(shooterControls.getManualHoodDown()) {
        m_hood.manualMoveHoodDown();
        System.out.println("HOOD DOWN");
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
    if(SmartDashboard.getBoolean(Constants.SmartDashboard.kTurretOverrideString, false)){
      //System.out.println("MANUAL TURRET ENABLED");
      turretOnTarget = true;
      if(shooterControls.getManualTurretLeft()) {
        System.out.print("TURNING LEFT");
        m_turret.turnTurret(0.5);
      } else if(shooterControls.getManualTurretRight()) {
        //System.out.println("TURNING RIGHT");
        m_turret.turnTurret(-0.5);
      } else {
        //System.out.println("NO TURN SIGNAL");
        m_turret.turnTurret(0);
      }
    } else if(m_vision.hasValidTarget()){
      //System.out.println("AUTOMATIC MODE, HAS TARGET");
      double turretTargetAngle = m_vision.getTx(); //calculate target turret angle from vision
      turretOnTarget = m_turret.getIsOnTarget();
      m_turret.driveToPos(turretTargetAngle);//turn turret to target angle
    } else {
      System.out.print("NO TARGET");
      m_turret.turnTurret(0);
    }
  }

  public void executeShooter() {
    if(SmartDashboard.getBoolean(Constants.SmartDashboard.kShooterOverrideString, false)) {
      speedOnTarget = true;
      double currentPowerPct = m_shooter.getSpeedPct();
      if(shooterControls.getShooterIncrease()) {
        currentPowerPct += 0.005; //go up by 10% every second held
        if(currentPowerPct > 1) {
          currentPowerPct = 1;
        }
        System.out.println("INCREASING SHOOTER" + currentPowerPct);
        m_shooter.setShooterPct(currentPowerPct);
      } else if (shooterControls.getShooterDecrease()) {
        currentPowerPct -= 0.005;
        if(currentPowerPct < 0) {
          currentPowerPct = 0;
        }
        System.out.println("DECREASING SHOOTER" + currentPowerPct);
        m_shooter.setShooterPct(currentPowerPct);
      } else {
        m_shooter.setShooterPct(currentPowerPct);
      }
    } else {
      int targetSpeed = 0; //TODO: calculate targetSpeed in shooter 
      speedOnTarget = Math.abs(m_shooter.getVelocity() - targetSpeed) < .5;
      //m_shooter.setSpeed(targetSpeed);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //System.out.println("MEGASHOOTER EXECUTE");

    //System.out.println("TURRET OVERRIDE=" + SmartDashboard.getBoolean(Constants.SmartDashboard.kTurretOverrideString, false));

    if(shooterControls.getShootPressed() || shooterControls.getReadyPressed()) {
      m_vision.setLEDMode(0);
      m_vision.updateLimelight();
      executeHood();
      executeTurret();
      executeShooter();
      if(getIsReady() && shooterControls.getShootPressed() && !SmartDashboard.getBoolean(Constants.SmartDashboard.kConveyorOverrideString, false)) {
        if(!shooterControls.getManualConveyorDown()) {
          m_conveyor.lifterUp();
        } else {
          m_conveyor.lifterDown();
        }
      } else if(shooterControls.getManualConveyorUp() && SmartDashboard.getBoolean(Constants.SmartDashboard.kConveyorOverrideString, false)) {
          m_conveyor.lifterUp();
      } else {
        m_conveyor.lifterStop();
      }
    } else {
      m_vision.setLEDMode(1);
      m_vision.updateLimelight();
      m_shooter.setShooterPct(0);
      m_turret.turnTurret(0);
    }
  }

  public boolean getIsReady() {
    return hoodOnTarget && turretOnTarget && speedOnTarget;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.print("Mega Shooter Stopped");
    m_hood.manualStopHoodMovement();
    m_conveyor.lifterStop();
    m_shooter.setShooterPct(0);
    m_turret.turnTurret(0);    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;  //never "done" because it is the default command
  }
}
