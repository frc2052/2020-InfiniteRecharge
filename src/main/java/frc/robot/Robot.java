/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SmartDashboardStrings;
import frc.robot.auto.AutoModeSelector;
import frc.robot.commands.LoggingCommand;
import frc.robot.lib.CsvLogger;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.PixyCamSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.PixyCamSubsystem.PixyBlock;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private Command m_loggingCommand;

  private RobotContainer m_robotContainer;

  VisionSubsystem vision = null;
  DriveTrainSubsystem driveTrain = null;
  PixyCamSubsystem pixyCam = null;



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
    SmartDashboard.putNumber(Constants.SmartDashboardStrings.kAutoDelay, 0);
    SmartDashboard.putNumber(Constants.SmartDashboardStrings.kTurretTrim, 0);
    SmartDashboard.putNumber(Constants.SmartDashboardStrings.kShooterVelocityOverride, 0);
    SmartDashboard.putBoolean(Constants.SmartDashboardStrings.kDisabledLimeLightOn, false);
    // SmartDashboard.putNumber("PID - P Value", 1.3);
    // SmartDashboard.putNumber("PID - I Value", 0);
    // SmartDashboard.putNumber("PID - D Value", 0.5);

    SmartDashboard.putBoolean(Constants.SmartDashboardStrings.kEnableLogging, true);
    SmartDashboard.putNumber(Constants.SmartDashboardStrings.kLogEveryXRequests, 25);

    m_robotContainer.turnLEDSOff();

    pixyCam = new PixyCamSubsystem();
  }

  public void idleShooterOn() {
    
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
    CsvLogger.close();
  }

  @Override
  public void disabledPeriodic() {
    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.changeLimeLight(SmartDashboard.getBoolean(Constants.SmartDashboardStrings.kDisabledLimeLightOn, false));
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    CsvLogger.init();
    CsvLogger.setLogEveryXRequests(getLoggingRequestPerXRequests());

    //Units.inchesToMeters(AutoModeSelector.getPosOnLineInches())
    m_robotContainer.resetEncoders();
    //Units.inchesToMeters(138), Units.inchesToMeters(-68)

    if (m_loggingCommand != null){ //still exists from last auto/teleop run
      m_loggingCommand.cancel();
    }
    m_loggingCommand = new LoggingCommand();
    m_loggingCommand.schedule();

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
    if (!CsvLogger.isLogOpen()) //if already open, we switched from auto to teleop
    {
      CsvLogger.init();
    }
    CsvLogger.setLogEveryXRequests(getLoggingRequestPerXRequests());


    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.setMegaShooterDefaultCommand(true);

    if (m_loggingCommand != null){ //still exists from last auto/teleop run
      m_loggingCommand.cancel();
    }
    m_loggingCommand = new LoggingCommand();
    m_loggingCommand.schedule();
    m_robotContainer.unlockElevator();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    ArrayList<PixyBlock> ballArray = pixyCam.read();
    ballArray = pixyCam.sortListByX(ballArray);
    /*ballArray.forEach((ball)->{
      System.out.println("Ball position x: " + ball.centerX)
    });*/
    
    
    if(ballArray != null && ballArray.size() >=3){
      PixyBlock firstBall = ballArray.get(0);
      PixyBlock secondBall = ballArray.get(1);
      PixyBlock thirdBall = ballArray.get(2);

      System.out.println("Ball one position x: " + firstBall.centerX + " y: " + firstBall.centerY + " height " + firstBall.height);
      System.out.println("Ball two position x: " + secondBall.centerX + " y: " + secondBall.centerY + " height " + secondBall.height);
      System.out.println("Ball three position x: " + thirdBall.centerX + " y: " + thirdBall.centerY + " height " + thirdBall.height);

    } else if (ballArray != null && ballArray.size() == 2){
      PixyBlock firstBall = ballArray.get(0);
      PixyBlock secondBall = ballArray.get(1);

      System.out.println("Ball one position x: " + firstBall.centerX + " y: " + firstBall.centerY + " height " + firstBall.height);
      System.out.println("Ball two position x: " + secondBall.centerX + " y: " + secondBall.centerY + " height" + secondBall.height);
    } else if (ballArray != null && ballArray.size() ==1){
      PixyBlock firstBall = ballArray.get(0);

      System.out.println("Ball one position x " + firstBall.centerX + " y: " + firstBall.centerX + " height" + firstBall.height);
    }  else if (ballArray != null && ballArray.size() ==0){
      System.out.println("size is zero");
    }
    else {
      System.out.println("Ball array is null");
    } 

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

  private int getLoggingRequestPerXRequests() {
    int x = (int)SmartDashboard.getNumber(Constants.SmartDashboardStrings.kLogEveryXRequests, 50);
    if (x < 1){
      SmartDashboard.putBoolean(Constants.SmartDashboardStrings.kEnableLogging, false);
      return 50;
    } else {
      return x;
    }
  }
}
