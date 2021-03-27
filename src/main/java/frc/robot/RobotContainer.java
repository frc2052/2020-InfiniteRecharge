/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.*;
import frc.robot.subsystems.PixyCamSubsystem.galacticSearchEnum;
import frc.robot.Constants.SmartDashboardStrings;
import frc.robot.auto.*;
import frc.robot.commands.*;


/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private DriveTrainSubsystem driveTrain = null;
  private IntakeSubsystem intake = null;
  private ShooterSubsystem shooter = null;
  private HoodSubsystem hood = null;
  private TurretSubsystem turret = null;
  private VisionSubsystem vision = null;
  private ConveyorSubsystem conveyor = null;
  //private ElevatorSubsystem elevator = null;

  private Joystick turnJoystick;
  private Joystick tankJoystick;
  private Joystick secondaryPanel;

  private ShooterControls shooterControls= null;
  private AutoShooterControls autoShooterControls = new AutoShooterControls();

  private MegaShooterCommand megaShooterCommand = null;
  private VisionTurretAdjustCommand visionTurretCommand = null;
  private ManualSpinUpCommand manualSpinUp = null;
  private PixyCamManualDriveCommand pixyCamManualDriveCommand = null;
  private SmartIntakeCommand smartIntakeCommand = null;
  private PixyCamSubsystem pixyCam = null;

  public RobotContainer() {
    driveTrain = new DriveTrainSubsystem();
    intake = new IntakeSubsystem();
    shooter = new ShooterSubsystem();
    hood = new HoodSubsystem();
    turret = new TurretSubsystem();
    vision = new VisionSubsystem();
    conveyor = new ConveyorSubsystem();
    //elevator = new ElevatorSubsystem();
    smartIntakeCommand = new SmartIntakeCommand(conveyor, intake);
    pixyCam = new PixyCamSubsystem();

    PowerDistributionPanel pdp = new PowerDistributionPanel();
    SmartDashboard.putData(pdp);

    visionTurretCommand = new VisionTurretAdjustCommand(vision, turret);
    manualSpinUp = new ManualSpinUpCommand(shooter);

    //svision.setLEDMode(1);

    configureTurnJoystick();
    configureTankJoystick();
    configureSecondaryPanel();

    shooterControls = new ShooterControls(turnJoystick, tankJoystick, secondaryPanel);

    megaShooterCommand = new MegaShooterCommand(shooter, vision, hood, turret, conveyor, shooterControls);

    pixyCamManualDriveCommand = new PixyCamManualDriveCommand(driveTrain, tankJoystick);

    driveDefaultCommand();
    // setMegaShooterDefaultCommand();

  }

  public void resetEncoders() {
    driveTrain.resetEncoders();
    //elevator.resetEncoder();
    hood.resetEncoder();
    turret.resetEncoder();
    shooter.resetEncoder();
  }

  // public void setMegaShooterDefaultCommand(boolean isTeleop) {
  //   //System.out.println("INITIALIZED mEGASHOOTER COMMAND");
  //   if(isTeleop) {
  //     shooter.setDefaultCommand(megaShooterCommand);
  //   } else {
  //     shooter.setDefaultCommand(null);
  //   }
  // }

  public void driveDefaultCommand() {
    driveTrain.setDefaultCommand(
      new RunCommand(
        () -> driveTrain.curvatureDrive(tankJoystick.getY(), -turnJoystick.getX(), turnJoystick.getRawButton(3)), 
        driveTrain
      )
    );
  }

  public void configureTurnJoystick() {
    turnJoystick = new Joystick(0);

    JoystickButton btnJL1 = new JoystickButton(turnJoystick, 1); 
    JoystickButton btnJL2 = new JoystickButton(turnJoystick, 2); //manual conveyor down
    JoystickButton btnJL3 = new JoystickButton(turnJoystick, 3); //quick turn
    JoystickButton btnJL4 = new JoystickButton(turnJoystick, 4);
    JoystickButton btnJL5 = new JoystickButton(turnJoystick, 5);
    JoystickButton btnJL6 = new JoystickButton(turnJoystick, 6);
    JoystickButton btnJL7 = new JoystickButton(turnJoystick, 7);
    JoystickButton btnJL8 = new JoystickButton(turnJoystick, 8);
    JoystickButton btnJL9 = new JoystickButton(turnJoystick, 9);
    JoystickButton btnJL10 = new JoystickButton(turnJoystick, 10);
    JoystickButton btnJL11 = new JoystickButton(turnJoystick, 11);

    //btnJL1.whileHeld(pixyCamManualDriveCommand);

    // btnJL3.whenPressed(() -> conveyor.setWantUp(true)); 
    // btnJL3.whenReleased(() -> conveyor.setWantUp(false));

    btnJL4.whileHeld(() -> conveyor.setWantActiveBalanceLeft(true)); 
    btnJL4.whenReleased(() -> conveyor.setWantActiveBalanceLeft(false));

    btnJL5.whileHeld(() -> conveyor.setWantActiveBalanceRight(true)); 
    btnJL5.whenReleased(() -> conveyor.setWantActiveBalanceRight(false)); 

    btnJL6.whenPressed(() -> {}); 
    btnJL6.whenReleased(() -> {}); 

    btnJL7.whenPressed(() -> {}); 
    btnJL7.whenReleased(() -> {}); 

    btnJL8.whenPressed(() -> {}); 
    btnJL8.whenReleased(() -> {}); 

    btnJL9.whenPressed(() -> {}); 
    btnJL9.whenReleased(() -> {}); 
    
    btnJL10.whenPressed(() -> {}); 
    btnJL10.whenReleased(() -> {});

    btnJL11.whenPressed(() -> {});  
    btnJL11.whenReleased(() -> {});
  }

  public void configureTankJoystick() {
    tankJoystick = new Joystick(1);

    JoystickButton btnJR1 = new JoystickButton(tankJoystick, 1); //shoot in megashooter
    JoystickButton btnJR2 = new JoystickButton(tankJoystick, 2);
    JoystickButton btnJR3 = new JoystickButton(tankJoystick, 3); //ready in megashooter
    JoystickButton btnJR4 = new JoystickButton(tankJoystick, 4); //shooter idle speed
    JoystickButton btnJR5 = new JoystickButton(tankJoystick, 5);
    JoystickButton btnJR6 = new JoystickButton(tankJoystick, 6);
    JoystickButton btnJR7 = new JoystickButton(tankJoystick, 7);
    JoystickButton btnJR8 = new JoystickButton(tankJoystick, 8);
    JoystickButton btnJR9 = new JoystickButton(tankJoystick, 9);
    JoystickButton btnJR10 = new JoystickButton(tankJoystick, 10);
    JoystickButton btnJR11 = new JoystickButton(tankJoystick, 11);

    btnJR2.whenPressed(() -> driveTrain.setHighGear(true)); //Shift speeds
    btnJR2.whenReleased(() -> driveTrain.setHighGear(false)); //stop shifting

    btnJR5.whenPressed(() -> {}); 
    btnJR5.whenReleased(() -> {}); 

    btnJR6.whenPressed(() ->  {});
    btnJR6.whenReleased(() ->  {}); 

    btnJR7.whenPressed(() -> {}); 
    btnJR7.whenReleased(() -> {});

    btnJR8.whenPressed(() -> {}); 
    btnJR8.whenReleased(() -> {});

    btnJR9.whenPressed(() -> {}); 
    btnJR9.whenReleased(() -> {});

    btnJR10.whenPressed(() -> {}); 
    btnJR10.whenReleased(() -> {});

    btnJR11.whenPressed(() -> {}); 
    btnJR11.whenReleased(() -> {});
  }

  public void configureSecondaryPanel() {
    secondaryPanel = new Joystick(2);

    JoystickButton btnJS1 = new JoystickButton(secondaryPanel, 1);
    JoystickButton btnJS2 = new JoystickButton(secondaryPanel, 2);
    JoystickButton btnJS3 = new JoystickButton(secondaryPanel, 3);
    JoystickButton btnJS4 = new JoystickButton(secondaryPanel, 4);
    JoystickButton btnJS5 = new JoystickButton(secondaryPanel, 5);
    JoystickButton btnJS6 = new JoystickButton(secondaryPanel, 6);
    JoystickButton btnJS7 = new JoystickButton(secondaryPanel, 7);
    JoystickButton btnJS8 = new JoystickButton(secondaryPanel, 8);
    JoystickButton btnJS9 = new JoystickButton(secondaryPanel, 9);
    JoystickButton btnJS10 = new JoystickButton(secondaryPanel, 10);
    JoystickButton btnJS11 = new JoystickButton(secondaryPanel, 11);
    JoystickButton btnJS12 = new JoystickButton(secondaryPanel, 12);

    btnJS1.whenPressed(() -> intake.armToggle());

    btnJS2.whenPressed(() -> hood.manualMoveHoodUp());
    btnJS2.whenReleased(() -> hood.manualStopHoodMovement());

    //btnJS3.whenPressed(() -> elevator.unlockElevator()); 

    //btnJS4.whenPressed(() -> elevator.setOverride(true));
    //btnJS4.whenReleased(() -> elevator.setOverride(false));

    //btnJS5.whenPressed(() -> elevator.lockElevator()); 

    //btnJS6.whileHeld(() -> intake.intakeIn()); 
    btnJS6.whileHeld(smartIntakeCommand); 
    btnJS6.whenReleased(() -> intake.intakeStop());

    btnJS7.whileHeld(() -> intake.intakeOut()); 
    btnJS7.whenReleased(() -> intake.intakeStop());

    btnJS8.whenPressed(() -> hood.manualMoveHoodDown()); 
    btnJS8.whenReleased(() -> hood.manualStopHoodMovement());

    // btnJS9.whenPressed(() -> turret.turnTurret(-0.5)); //add to megashooter
    // btnJS9.whenReleased(() -> turret.turnTurret(0));
    
    // btnJS10.whenPressed(() -> turret.turnTurret(0.5));
    // btnJS10.whenReleased(() -> turret.turnTurret(0));

    //btnJS11.whileHeld(() -> elevator.manualDown());
    //btnJS11.whenReleased(() -> elevator.manualStop());

    //btnJS12.whileHeld(() -> elevator.manualUp());
    //btnJS12.whenReleased(() -> elevator.manualStop());

  }
  
  public void turnLEDSOff() {
    limelightWasOn = true;
    changeLimeLight(false);
//    vision.setLEDMode(1);
  }

  public Command drivePathCommand() {
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(Constants.DriveTrain.ksVolts,
                                       Constants.DriveTrain.kvVoltSecondsPerMeter,
                                       Constants.DriveTrain.kaVoltSecondsSquaredPerMeter),
            Constants.DriveTrain.kinematics,
            10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(Constants.Autonomous.maxVelocity,
                             Constants.Autonomous.maxAcceleration)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.DriveTrain.kinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))),
      // Pass through these two interior waypoints, making an 's' curve path
      //new ArrayList<Translation2d>(),
      List.of(
          new Translation2d(Units.feetToMeters(2), 0),
          new Translation2d(Units.feetToMeters(3), 0)
      ),
      // End 3 meters straight ahead of where we started, facing forward
      new Pose2d(Units.feetToMeters(5), 0, new Rotation2d(Math.toRadians(0))),
        // Pass config
        config
    );

    PIDController leftController = new PIDController(41.5, 0, 0);
    PIDController rightController = new PIDController(41.5, 0, 0);

    RamseteCommand ramseteCommand = new RamseteCommand(
        exampleTrajectory,
        driveTrain::getPose,
        new RamseteController(Constants.DriveTrain.kRamseteB, Constants.DriveTrain.kRamseteZeta),
        new SimpleMotorFeedforward(Constants.DriveTrain.ksVolts,
                                   Constants.DriveTrain.kvVoltSecondsPerMeter,
                                   Constants.DriveTrain.kaVoltSecondsSquaredPerMeter),
        Constants.DriveTrain.kinematics,
        driveTrain::getWheelSpeeds,
        leftController,
        rightController,
        // RamseteCommand passes volts to the callback
        (leftVolts, rightVolts)-> {
          driveTrain.tankDriveVolts(leftVolts, rightVolts);

          SmartDashboard.putNumber("leftMeasurement", driveTrain.getWheelSpeeds().leftMetersPerSecond);
          SmartDashboard.putNumber("leftReference", leftController.getSetpoint());

          SmartDashboard.putNumber("rightMeasurement", driveTrain.getWheelSpeeds().rightMetersPerSecond);
          SmartDashboard.putNumber("rightReference", rightController.getSetpoint());
        },
        driveTrain
    );


    return ramseteCommand.andThen(() -> driveTrain.tankDriveVolts(0, 0));
  }

  public void putToSmartDashboard() {
    driveTrain.putToSmartDashboard();
    //elevator.printEncoderPos();
    hood.putEncoderToShuffleboard();
    vision.putDistanceToSmartDashboard();
    turret.printEncoderPos();
    shooter.putToSmartDashboard();
  }

  public void unlockElevator() {
    //elevator.unlockElevator();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    //return null;

    //return drivePathCommand();

    //return drivePathCommand();

    // double x = 0;
    // double y = 619.25;

    // switch (pos.getString("middle")){
    //   case "Middle":
    //     y = 619.25;
    //     break;
    //   case "Forward":
    //     y= 619.25 + 19;
    //     break;
    //   case "Back":

    //     y = 619.25 - 19;
    //     break;
    // }

    // switch (isLR.getString("Right")){
    //   case "Right":
    //     x = 203.25 - measurement.getDouble(0);
    //     break;
    //   case "Left":
    //     x= measurement.getDouble(0);
    //     break;
    // }
    // driveTrain.setOdometry(x, y);

    double delay = SmartDashboard.getNumber(Constants.SmartDashboardStrings.kAutoDelay, 0);

    switch(AutoModeSelector.getSelectedAuto()) { 
      case DRIVE:
        driveTrain.setOdometry(Units.inchesToMeters(138), Units.inchesToMeters(0));
        DriveCommand drive = new DriveCommand(driveTrain);
        return drive;
      case DM:
        return null;
      // case LSG3:
      //   StartLeftGenerator3Command leftGenerator3 = new StartLeftGenerator3Command(driveTrain, shooter, intake, vision, hood, turret, conveyor, delay, autoShooterControls);
      //   return leftGenerator3;
      // case LSG5:
      //   StartLeftShoot5Command leftShoot5 = new StartLeftShoot5Command(driveTrain, shooter, intake, vision, hood, turret, conveyor, delay, autoShooterControls);
      //   return leftShoot5;
      case LST2:
        driveTrain.setOdometry(Units.inchesToMeters(138), Units.inchesToMeters(-220));
        StartLeftTrench2Command leftTrench2 = new StartLeftTrench2Command(driveTrain, shooter, intake, vision, hood, turret, conveyor, delay, autoShooterControls);
        return leftTrench2;
      case RST3:
        driveTrain.setOdometry(Units.inchesToMeters(138), Units.inchesToMeters(68));
        StartRightTrench3Command rightTrench3 = new StartRightTrench3Command(driveTrain, shooter, intake, vision, hood, turret, conveyor, delay, autoShooterControls);
        return rightTrench3;
      case RST32:
        driveTrain.setOdometry(Units.inchesToMeters(138), Units.inchesToMeters(68));
        StartRightShootTrench3Command rightShootTrench3 = new StartRightShootTrench3Command(driveTrain, shooter, intake, vision, hood, turret, conveyor, delay, autoShooterControls);
        return rightShootTrench3;
      case CSG2:
        driveTrain.setOdometry(Units.inchesToMeters(138),0);
        StartCenterGenerator2Command startGen3 = new StartCenterGenerator2Command(driveTrain, shooter, intake, vision, hood, turret, conveyor, delay, autoShooterControls);
        return startGen3;
      case CS:
        driveTrain.setOdometry(Units.inchesToMeters(138), 0);
        CenterShootDriveParkCommand centerShootDrivePark = new CenterShootDriveParkCommand(driveTrain, shooter, vision, hood, turret, conveyor, delay, autoShooterControls);
        return centerShootDrivePark;
      case GS:
        GalacticSearchARedCommand galacticSearchARed = new GalacticSearchARedCommand(driveTrain, intake, vision);
        GalacticSearchABlueCommand galacticSearchABlue = new GalacticSearchABlueCommand(driveTrain, intake, vision);
        GalacticSearchBRedCommand galacticSearchBRed = new GalacticSearchBRedCommand(driveTrain, intake, vision);
        GalacticSearchBBlueCommand galacticSearchBBlue = new GalacticSearchBBlueCommand(driveTrain, intake, vision);

        //driveTrain.setOdometry(Units.inchesToMeters(48), Units.inchesToMeters(60));
        //return galacticSearchBBlue;
        
        
        if(pixyCam.getGSPath() == galacticSearchEnum.ARED){
          driveTrain.setOdometry(Units.inchesToMeters(48), Units.inchesToMeters(120));
          return galacticSearchARed;
        } else if (pixyCam.getGSPath() == galacticSearchEnum.ABLUE){
          driveTrain.setOdometry(Units.inchesToMeters(48), Units.inchesToMeters(30));
          return galacticSearchABlue;

        } else if(pixyCam.getGSPath() == galacticSearchEnum.BRED){
          driveTrain.setOdometry(Units.inchesToMeters(48), Units.inchesToMeters(120));
          return galacticSearchBRed;

        } else if(pixyCam.getGSPath() == galacticSearchEnum.BBLUE){
          driveTrain.setOdometry(Units.inchesToMeters(48), Units.inchesToMeters(60));
          return galacticSearchBBlue;

        } else{
          return null;
        }

        // driveTrain.setOdometry(Units.inchesToMeters(48), Units.inchesToMeters(90)); //Red A start pos
        // driveTrain.setOdometry(Units.inchesToMeters(48), Units.inchesToMeters(30)); //Blue A start pos
        // driveTrain.setOdometry(Units.inchesToMeters(48), Units.inchesToMeters(120)); //Red B start pos
        // driveTrain.setOdometry(Units.inchesToMeters(48), Units.inchesToMeters(60)); //Blue B start pos
      case ANB:
        driveTrain.setOdometry(Units.inchesToMeters(43), Units.inchesToMeters(90));
        AutoNavBarrelCommand autoNavBarrel = new AutoNavBarrelCommand(driveTrain);
        return autoNavBarrel;
      case ANS:
        driveTrain.setOdometry(Units.inchesToMeters(43), Units.inchesToMeters(30));
        AutoNavSlalomCommand autoNavSlalom = new AutoNavSlalomCommand(driveTrain);
        return autoNavSlalom;
      default:
        return null; 
    }
  }

  private boolean limelightWasOn = true;
  public void changeLimeLight(boolean turnOn){
    if (vision != null)
    {
      if (limelightWasOn != turnOn) {
        if (turnOn){
          System.out.println("TURNING LIMELIGHT --- ON");
          vision.setLEDMode(3);
        } else {
          System.out.println("TURNING LIMELIGHT --- OFF");
          vision.setLEDMode(1);
        }
        limelightWasOn = turnOn;
      }
    }
  }
}
