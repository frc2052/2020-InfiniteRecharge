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
import frc.robot.subsystems.*;
import frc.robot.vision.VisionCalculator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MegaShooterCommand extends CommandBase {
  private ShooterSubsystem m_shooter;
  protected VisionSubsystem m_vision; //autoShooter needs access
  private HoodSubsystem m_hood;
  protected TurretSubsystem m_turret; //autoShooter needs access
  private ConveyorSubsystem m_conveyor;

  private IShooterControls shooterControls;
  private VisionCalculator visionCalculator;

  private boolean hoodOnTarget = false;
  private boolean turretOnTarget = false;
  private boolean speedOnTarget = false;
  private boolean m_isFalconShooter = true;

  public MegaShooterCommand(ShooterSubsystem shooter, VisionSubsystem vision, HoodSubsystem hood, TurretSubsystem turret, ConveyorSubsystem conveyor, IShooterControls controls) {
    m_shooter = shooter;
    m_vision = vision;
    m_hood = hood;
    m_turret = turret;
    m_conveyor = conveyor;
    m_isFalconShooter = (m_shooter instanceof FalconShooterSubsystem);

    shooterControls = controls;
    visionCalculator = new VisionCalculator();

    addRequirements(shooter, hood, turret);    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //System.out.println("*************** MEGA SHOOTER INIT");
  }

  public void executeHood(int inchesAway) {
    if(SmartDashboard.getBoolean(Constants.SmartDashboardStrings.kHoodOverrideString, false)) {
      //Hood is in manual mode, assume hood is on target
      hoodOnTarget = true;
    } else {
      //Hood is in automatic mode      
      int targetTicks = visionCalculator.distanceToTicks(inchesAway);
      int tickTrim = (int)SmartDashboard.getNumber(Constants.SmartDashboardStrings.kHoodTrim, 0);
      targetTicks = targetTicks + tickTrim;
      System.out.println("DISTANE CALCULATED====" + inchesAway + "   HOOD TARGET TICKS====" + targetTicks);
      //calculate the hood angle from the hood system
      m_hood.driveToEncoderPos(targetTicks);
      //System.out.println("TARGET TICKS HOOD====" + hoodTargetTicks);
      //System.out.println("HOOD CURRET ANGLE====" + hoodCurrentTicks);
      hoodOnTarget = m_hood.isOnTarget(); // Math.abs(hoodTargetTicks - hoodCurrentTicks) < 5000;
      //turn hood to target angle 
    }
  }

  public void executeTurret() {
    if(SmartDashboard.getBoolean(Constants.SmartDashboardStrings.kTurretOverrideString, false)){
      //Turret in manual mode - assume on target
      turretOnTarget = true;
      m_turret.turnTurret(0); //stop the turret from turning, if we switch ot manual mode while turning, it will keep turning if we don't do this
    } else if(m_vision.hasValidTarget()){  
      //Turret in automatic mode
      double adjustDegrees = SmartDashboard.getNumber(Constants.SmartDashboardStrings.kTurretTrim, 0);
      double turretTargetAngle = m_vision.getTx() + adjustDegrees; //calculate target turret angle from vision
      //System.out.println("AUTOMATIC MODE, HAS TARGET TURRET TARGET ANGLE---" + turretTargetAngle);
      turretOnTarget = m_turret.getIsOnTarget();
      m_turret.driveToPos(turretTargetAngle);//turn turret to target angle
    } else { //not in manual mode, doesn't see target
      //System.out.print("NO TARGET");
      m_turret.turnTurret(0);
      turretOnTarget = false;
    }
  }

  public void executeShooter(int inchesAway) {
      double targetSpeed;

      if(SmartDashboard.getNumber(Constants.SmartDashboardStrings.kShooterVelocityOverride, 0) == 0) {
        //no shooter override, use standard logic
        targetSpeed = Constants.Shooter.kShooterTargetVelocity;
        if (inchesAway > 30 * 12 && m_isFalconShooter) //more than 30 feet away, and shooter is using falcons
        {
          targetSpeed = targetSpeed * 1.15; //increase speed by 15%
        }
      } else {
        targetSpeed = SmartDashboard.getNumber(Constants.SmartDashboardStrings.kShooterVelocityOverride, 0);
      }

      // speedOnTarget = Math.abs(m_shooter.getVelocity() - targetSpeed) < .5;
      m_shooter.setShooterVelocity(targetSpeed);
      speedOnTarget = m_shooter.getIsOnTarget();      
//      SmartDashboard.putNumber("SHOOTER VELOCITY", m_shooter.getVelocityTicks());
    }

  public boolean getTotalManualMode() {
    return SmartDashboard.getBoolean(Constants.SmartDashboardStrings.kShooterOverrideString, false) 
      && SmartDashboard.getBoolean(Constants.SmartDashboardStrings.kTurretOverrideString, false) 
      && SmartDashboard.getBoolean(Constants.SmartDashboardStrings.kHoodOverrideString, false);
  }

  public void executeConveyor() {
    if(shooterControls.getManualConveyorDown()) {
      m_conveyor.setWantDown(true);
    } else {
      m_conveyor.setWantDown(false);
      if(getIsReady() && shooterControls.getShootPressed() && !getTotalManualMode()) {        
        m_conveyor.setWantUp(true);
      } else if(shooterControls.getManualConveyorUp()) {
        m_conveyor.setWantManualUp(true);
      } else {
        m_conveyor.setWantUp(false);
        m_conveyor.setWantManualUp(false);
      }
    }
  }

  public void executeManual() {
    if(shooterControls.getManualTurretLeft()) {
      //System.out.print("TURNING LEFT");
      m_turret.turnTurret(0.5);
    } else if(shooterControls.getManualTurretRight()) {
      //System.out.println("TURNING RIGHT");
      m_turret.turnTurret(-0.5);
    } else {
      m_turret.turnTurret(0);
    }

    if(shooterControls.getManualHoodUp()) {
      //System.out.println("HOOD UP");
      m_hood.manualMoveHoodUp();
    } else if(shooterControls.getManualHoodDown()) {
      m_hood.manualMoveHoodDown();
      //System.out.println("HOOD DOWN");
    } else {
      m_hood.manualStopHoodMovement();
    }

    boolean overrideShooter = SmartDashboard.getBoolean(Constants.SmartDashboardStrings.kShooterOverrideString, false);
    if(overrideShooter) {
      System.out.println("Increase + " + shooterControls.getShooterIncrease());
      speedOnTarget = true;
      double currentPowerPct = m_shooter.getSpeedPct();
      if(shooterControls.getShooterIncrease()) {
        currentPowerPct += 0.01; //go up by this percent every second held
        if(currentPowerPct > 1) {
          currentPowerPct = 1;
        }
        //System.out.println("INCREASING SHOOTER" + currentPowerPct);
        m_shooter.setShooterPct(currentPowerPct);
      } else if (shooterControls.getShooterDecrease()) {
        currentPowerPct -= 0.01;
        if(currentPowerPct < 0) {
          currentPowerPct = 0;
        }
        //System.out.println("DECREASING SHOOTER" + currentPowerPct);
        m_shooter.setShooterPct(currentPowerPct);
      } else {
        //System.out.println("Pct Shooter " + currentPowerPct);
        m_shooter.setShooterPct(currentPowerPct);
      }
      System.out.println("SHOOTER POWER PCT=====" + currentPowerPct);
    } else { //no shooter override
      if (!shooterIdleIsOn) {
        m_shooter.setShooterPct(0);
      } else {
        m_shooter.setShooterPct(.5);
      }
    }
  }

  private boolean wasToggleIdleLastPressed = false;
  private boolean shooterIdleIsOn = true;

  public void toggleIdleShooter() {
    if (shooterControls == null){
      System.err.println("SHOOTER CONTROLS NULL");
    }
    if (shooterControls.getIdleShooterToggle() && !wasToggleIdleLastPressed) { //first time we have seen it pressed since last check
      if (shooterIdleIsOn) {
        shooterIdleIsOn = false;
      } else {
        shooterIdleIsOn = true;
      }
    }
    wasToggleIdleLastPressed = shooterControls.getIdleShooterToggle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    toggleIdleShooter();

    if(shooterControls.getShootPressed() || shooterControls.getReadyPressed()) {
      m_vision.setLEDMode(3);
      m_vision.updateLimelight();
      int inches = visionCalculator.getDistance(m_vision.getTy(), m_vision.getTa(), 0, m_vision.getThor());
      executeHood(inches);
      executeTurret();
      executeShooter(inches);
      executeConveyor();
    } else {
      m_vision.setLEDMode(1);
      m_vision.updateLimelight(); 
      m_conveyor.setWantUp(false);
      m_conveyor.setWantDown(false);

      executeManual();
    }
  }

  public boolean getIsReady() {
    //System.out.println("HoodReady: " + hoodOnTarget + "  TurretReady: " + turretOnTarget + " SpeedReady: " + speedOnTarget);
    SmartDashboard.putBoolean("HoodOnTarget", hoodOnTarget);
    SmartDashboard.putBoolean("ShooterOnTarget", speedOnTarget);
    SmartDashboard.putBoolean("TurretOnTarget", turretOnTarget);
    return hoodOnTarget && turretOnTarget && speedOnTarget;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.print("Mega Shooter Stopped");
    m_hood.manualStopHoodMovement();
    m_conveyor.setWantDown(false);
    m_conveyor.setWantUp(false);
    m_shooter.setShooterPct(0);
    m_turret.turnTurret(0);    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;  //never "done" because it is the default command
  }
}
