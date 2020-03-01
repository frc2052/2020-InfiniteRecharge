/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.auto.AutoModeSelector;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  VisionSubsystem vision = null;
  DriveTrainSubsystem driveTrain = null;



  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    AutoModeSelector.putToShuffleBoard();
    SmartDashboard.putBoolean(Constants.SmartDashboardStrings.kShooterOverrideString, false);
    SmartDashboard.putBoolean(Constants.SmartDashboardStrings.kHoodOverrideString, false);
    SmartDashboard.putBoolean(Constants.SmartDashboardStrings.kTurretOverrideString, false);
    SmartDashboard.putBoolean(Constants.SmartDashboardStrings.kAutoBumpString, false);
    SmartDashboard.putNumber(Constants.SmartDashboardStrings.kTimeToShoot, 0);
    SmartDashboard.putNumber("Auto Delay", 0);
    SmartDashboard.putNumber(Constants.SmartDashboardStrings.kTurretTrim, 0);
    SmartDashboard.putNumber(Constants.SmartDashboardStrings.kShooterVelocityOverride, 0);
    //ShuffleboardTab tab = .getTab("manageAuto");
    // NetworkTableEntry pos =
    //         tab.add("Position On Line", "Middle")
    //                 .getEntry();
    // NetworkTableEntry isLR =
    //         tab.add("Measuring from Left Or Right", "Right")
    //                 .getEntry();
    // NetworkTableEntry measurement =
    //         tab.add("Distance", "0")
    //                 .getEntry();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    m_robotContainer.putToSmartDashboard();
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void disabledPeriodic() {
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    //Units.inchesToMeters(AutoModeSelector.getPosOnLineInches())
    m_robotContainer.resetEncoders();
    //Units.inchesToMeters(138), Units.inchesToMeters(-68)
    m_robotContainer.setOdometry(Units.inchesToMeters(138), Units.inchesToMeters(0));
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    
    //m_robotContainer.setMegaShooterDefaultCommand(false);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.setMegaShooterDefaultCommand(true);
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    
  }

  @Override
  public void testInit() {
    m_robotContainer.resetEncoders();
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
