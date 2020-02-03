/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
//import frc.robot.subsystems.HoodSubsystem.anglePresetEnum;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.*;
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
  private final ShuffleboardTab tab = Shuffleboard.getTab("manageAuto");

  private NetworkTableEntry autoDelay =
          tab.add("Auto Delay", 0)
                  .getEntry();


  private DriveTrainSubsystem driveTrain = null;
  private IntakeSubsystem intake = null;
  private ShooterSubsystem shooter = null;
  private HoodSubsystem hood = null;
  private TurretSubsystem turret = null;
  private VisionSubsystem vision = null;
  private ConveyorSubsystem conveyor = null;

  private Joystick turnJoystick;
  private Joystick tankJoystick;
  private Joystick secondaryPanel;

  JoystickButton btnJL1;
  JoystickButton btnJL2;
  JoystickButton btnJL3;
  JoystickButton btnJL4;
  JoystickButton btnJL5;
  JoystickButton btnJL6;
  JoystickButton btnJL7;
  JoystickButton btnJL8;
  JoystickButton btnJL9;
  JoystickButton btnJL10;
  JoystickButton btnJL11;

  JoystickButton btnJR1;
  JoystickButton btnJR2;
  JoystickButton btnJR3;
  JoystickButton btnJR4;
  JoystickButton btnJR5;
  JoystickButton btnJR6;
  JoystickButton btnJR7;
  JoystickButton btnJR8;
  JoystickButton btnJR9;
  JoystickButton btnJR10;
  JoystickButton btnJR11;

  JoystickButton btnJS1;
  JoystickButton btnJS2;
  JoystickButton btnJS3;
  JoystickButton btnJS4;
  JoystickButton btnJS5;
  JoystickButton btnJS6;
  JoystickButton btnJS7;
  JoystickButton btnJS8;
  JoystickButton btnJS9;
  JoystickButton btnJS10;
  JoystickButton btnJS11;
  JoystickButton btnJS12;
  
  private MegaShooterCommand megaShooterCommand = null;
  private VisionTurretAdjustCommand visionTurretCommand = null;

  public RobotContainer() {
    driveTrain = new DriveTrainSubsystem();
    intake = new IntakeSubsystem();
    shooter = new ShooterSubsystem();
    hood = new HoodSubsystem();
    turret = new TurretSubsystem();
    vision = new VisionSubsystem();
    conveyor = new ConveyorSubsystem();

    megaShooterCommand = new MegaShooterCommand(shooter, vision, hood, turret, conveyor);
    visionTurretCommand = new VisionTurretAdjustCommand(vision, turret);

    configureTurnJoystick();
    configureTankJoystick();
    configureSecondaryPanel();

    driveDefaultCommand();
    megaShooterDefaultCommand();
  }

  public void megaShooterDefaultCommand() {
    shooter.setDefaultCommand(
      new RunCommand(
        () -> megaShooterCommand.update(btnJL1, btnJL2, btnJL3, btnJL4, btnJL5, btnJL6, btnJL7, btnJL8, btnJL9)
      )
    );
  }

  public void driveDefaultCommand() {
    driveTrain.setDefaultCommand(
      // Default to basic tank drive
      new RunCommand(
        () -> driveTrain.tankDrive(tankJoystick.getY(), turnJoystick.getY() ), 
        driveTrain
      )
    );
  }


  public void configureTurnJoystick() {
    turnJoystick = new Joystick(0);

    btnJL1 = new JoystickButton(turnJoystick, 1);
    btnJL2 = new JoystickButton(turnJoystick, 2);
    JoystickButton btnJL3 = new JoystickButton(turnJoystick, 3);
    JoystickButton btnJL4 = new JoystickButton(turnJoystick, 4);
    JoystickButton btnJL5 = new JoystickButton(turnJoystick, 5);
    JoystickButton btnJL6 = new JoystickButton(turnJoystick, 6);
    JoystickButton btnJL7 = new JoystickButton(turnJoystick, 7);
    JoystickButton btnJL8 = new JoystickButton(turnJoystick, 8);
    JoystickButton btnJL9 = new JoystickButton(turnJoystick, 9);
    JoystickButton btnJL10 = new JoystickButton(turnJoystick, 10);
    JoystickButton btnJL11 = new JoystickButton(turnJoystick, 11);

    //buttons 1-9 are already in use in mega shooter
    btnJL1.whenPressed(() -> {});
    btnJL1.whenReleased(() -> {});

    btnJL2.whenPressed(() -> {}); 
    btnJL2.whenReleased(() -> {});

    btnJL3.whenPressed(() -> {}); 
    btnJL3.whenReleased(() -> {}); 

    btnJL4.whenPressed(() -> {}); 
    btnJL4.whenReleased(() -> {}); 

    btnJL5.whenPressed(() -> {}); 
    btnJL5.whenReleased(() -> {}); 

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

    JoystickButton btnJR1 = new JoystickButton(tankJoystick, 1);
    JoystickButton btnJR2 = new JoystickButton(tankJoystick, 2);
    JoystickButton btnJR3 = new JoystickButton(tankJoystick, 3);
    JoystickButton btnJR4 = new JoystickButton(tankJoystick, 4);
    JoystickButton btnJR5 = new JoystickButton(tankJoystick, 5);
    JoystickButton btnJR6 = new JoystickButton(tankJoystick, 6);
    JoystickButton btnJR7 = new JoystickButton(tankJoystick, 7);
    JoystickButton btnJR8 = new JoystickButton(tankJoystick, 8);
    JoystickButton btnJR9 = new JoystickButton(tankJoystick, 9);
    JoystickButton btnJR10 = new JoystickButton(tankJoystick, 10);
    JoystickButton btnJR11 = new JoystickButton(tankJoystick, 11);

    btnJR1.whenPressed(() -> shooter.setSpeed(Constants.Shooter.kShooterSpeedRPS));
    btnJR1.whenReleased(() -> {});

    btnJR2.whenPressed(() -> turret.printEncoderPos()); //Shift speeds
    btnJR2.whenReleased(() -> {}); //stop shifting

    btnJR3.whenPressed(() -> turret.turnTurret(-0.5)); 
    btnJR3.whenReleased(() -> turret.turnTurret(0.0)); 

    btnJR4.whileHeld(visionTurretCommand);
    btnJR4.whenReleased(() -> visionTurretCommand.end(true)); 

    btnJR5.whenPressed(() -> turret.turnTurret(0.5)); 
    btnJR5.whenReleased(() -> turret.turnTurret(0.0)); 

    btnJR6.whenPressed(() ->  driveTrain.setHighGear(true));
    btnJR6.whenReleased(() ->  driveTrain.setHighGear(false)); 

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

    btnJS1 = new JoystickButton(secondaryPanel, 1);
    btnJS2 = new JoystickButton(secondaryPanel, 2);
    btnJS3 = new JoystickButton(secondaryPanel, 3);
    btnJS4 = new JoystickButton(secondaryPanel, 4);
    btnJS5 = new JoystickButton(secondaryPanel, 5);
    btnJS6 = new JoystickButton(secondaryPanel, 6);
    btnJS7 = new JoystickButton(secondaryPanel, 7);
    btnJS8 = new JoystickButton(secondaryPanel, 8);
    btnJS9 = new JoystickButton(secondaryPanel, 9);
    btnJS10 = new JoystickButton(secondaryPanel, 10);
    btnJS11 = new JoystickButton(secondaryPanel, 11);
    btnJS12 = new JoystickButton(secondaryPanel, 12);

    btnJS1.whenPressed(() -> intake.armToggle());
    btnJS2.whenPressed(() -> intake.intakeIn());

    btnJS2.whenReleased(() -> intake.intakeStop());
    btnJS3.whenPressed(() -> intake.intakeOut());

    btnJS3.whenReleased(() -> intake.intakeStop());
    btnJS4.whenPressed(() -> {});

    btnJS5.whenPressed(() -> {}); 
    btnJS5.whenReleased(() -> {});

    btnJS6.whenPressed(() -> {}); 
    btnJS6.whenReleased(() -> {});

    btnJS7.whenPressed(() -> {});
    btnJS7.whenReleased(() -> {});

    btnJS8.whenPressed(() -> {});
    btnJS8.whenReleased(() -> {});

    btnJS9.whenPressed(() -> {});
    btnJS9.whenReleased(() -> {});
    
    btnJS10.whenPressed(() -> {});
    btnJS10.whenReleased(() -> {});

    btnJS11.whenPressed(() -> {});
    btnJS11.whenReleased(() -> {});

    btnJS12.whenPressed(() -> {});
    btnJS12.whenReleased(() -> {});
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    final ShuffleboardTab tab = Shuffleboard.getTab("manageAuto");

    double x = 0;
    double y = 619.25;

    NetworkTableEntry pos =
            tab.add("Position On Line", "Middle")
                    .getEntry();
    NetworkTableEntry isLR =
            tab.add("Measuring from Left Or Right", "Right")
                    .getEntry();
    NetworkTableEntry measurement =
            tab.add("Distance", "0")
                    .getEntry();
    switch (pos.getString("middle")){
      case "Middle":
        y = 619.25;
        break;
      case "Forward":
        y= 619.25 + 19;
        break;
      case "Back":

        y = 619.25 - 19;
        break;
    }

    switch (isLR.getString("Right")){
      case "Right":
        x = 203.25 - measurement.getDouble(0);
        break;
      case "Left":
        x= measurement.getDouble(0);
        break;
    }
    driveTrain.setOdometry(x, y);

    switch(AutoModeSelector.getSelectedAuto()) { //TODO: update this list once we have more autos
      case LSG3:
        StartLeftGenerator3Command leftGenerator3 = new StartLeftGenerator3Command(driveTrain, shooter, intake, vision, hood, turret, conveyor, autoDelay.getDouble(0));
        return leftGenerator3;
      case LSG5:
        StartLeftShoot5Command leftShoot5 = new StartLeftShoot5Command(driveTrain, shooter, intake, vision, hood, turret, conveyor, autoDelay.getDouble(0));
        return leftShoot5;
      case LST2:
        StartLeftTrench2Command leftTrench2 = new StartLeftTrench2Command(driveTrain, shooter, intake, vision, hood, turret, conveyor, autoDelay.getDouble(0));
        return leftTrench2;
      case RST3:
        StartRightTrench3Command rightTrench3 = new StartRightTrench3Command(driveTrain, shooter, intake, vision, hood, turret, conveyor, autoDelay.getDouble(0));
        return rightTrench3;
      case CSG3:
        return null;
      case CS:
        CenterShootDriveParkCommand centerShootDrivePark = new CenterShootDriveParkCommand(driveTrain, shooter, vision, hood, turret, conveyor, autoDelay.getDouble(0));
        return centerShootDrivePark;
      default:
        return null; 
    }
  }
}
