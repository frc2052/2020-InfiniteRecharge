/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
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

  private DriveTrainSubsystem driveTrain = null;
  private IntakeSubsystem intake = null;
  private ShooterSubsystem shooter = null;
  private HoodSubsystem hood = null;
  private TurretSubsystem turret = null;
  private VisionSubsystem vision = null;
  private ConveyorSubsystem conveyor = null;
  private ElevatorSubsystem elevator = null;
  private ActiveBalanceSubsytem activeBalance = null;

  private Joystick turnJoystick;
  private Joystick tankJoystick;
  private Joystick secondaryPanel;

  private MegaShooterCommand megaShooterCommand = null;
  private VisionTurretAdjustCommand visionTurretCommand = null;
  private PixyCamManualDriveCommand pixyCamManualDriveCommand = null;

  public RobotContainer() {
    driveTrain = new DriveTrainSubsystem();
    intake = new IntakeSubsystem();
    shooter = new ShooterSubsystem();
    hood = new HoodSubsystem();
    turret = new TurretSubsystem();
    vision = new VisionSubsystem();
    conveyor = new ConveyorSubsystem();
    elevator = new ElevatorSubsystem();
    activeBalance = new ActiveBalanceSubsytem();

    megaShooterCommand = new MegaShooterCommand(shooter, vision, hood, turret, conveyor);
    visionTurretCommand = new VisionTurretAdjustCommand(vision, turret);

    configureTurnJoystick();
    configureTankJoystick();
    configureSecondaryPanel();

    pixyCamManualDriveCommand = new PixyCamManualDriveCommand(driveTrain, tankJoystick);

    driveDefaultCommand();
    megaShooterDefaultCommand();
  }

  public void megaShooterDefaultCommand() {
    shooter.setDefaultCommand(
      new RunCommand(
        () -> megaShooterCommand.update(secondaryPanel.getRawButton(5), secondaryPanel.getRawButton(3), secondaryPanel.getRawButton(2), secondaryPanel.getRawButton(8), secondaryPanel.getRawButton(9), secondaryPanel.getRawButton(10), tankJoystick.getRawButton(3), tankJoystick.getTrigger(), turnJoystick.getRawButton(2), turnJoystick.getRawButton(3))
      )
    );
  }

  public void driveDefaultCommand() {
    driveTrain.setDefaultCommand(
      // Default to basic tank drive
      new RunCommand(
        () -> driveTrain.arcadeDrive(tankJoystick.getY(), turnJoystick.getX() ), 
        driveTrain
      )
    );
  }


  public void configureTurnJoystick() {
    turnJoystick = new Joystick(0);

    JoystickButton btnJL1 = new JoystickButton(turnJoystick, 1);
    JoystickButton btnJL2 = new JoystickButton(turnJoystick, 2);
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
    btnJL1.whileHeld(pixyCamManualDriveCommand);
    
    btnJL2.whenPressed(() -> conveyor.lifterDown()); //replace this in megashooter to test it
    btnJL2.whenReleased(() -> conveyor.lifterStop());

    btnJL3.whenPressed(() -> conveyor.lifterUp()); 
    btnJL3.whenReleased(() -> conveyor.lifterStop()); 

    btnJL4.whenPressed(() -> activeBalance.manualLeft()); 
    btnJL4.whenReleased(() -> activeBalance.manualStop()); 

    btnJL5.whenPressed(() -> activeBalance.manualRight()); 
    btnJL5.whenReleased(() -> activeBalance.manualStop()); 

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

    btnJR1.whenPressed(() -> {}); //shoot in megashooter
    btnJR1.whenReleased(() -> {});

    btnJR2.whenPressed(() -> driveTrain.setHighGear(true)); //Shift speeds
    btnJR2.whenReleased(() -> driveTrain.setHighGear(false)); //stop shifting

    btnJR3.whenPressed(() -> {}); //ready in megashooter
    btnJR3.whenReleased(() -> {}); 

    btnJR4.whileHeld(() -> {});
    btnJR4.whenReleased(() -> {}); 

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

    btnJS3.whenPressed(() -> {}); //manual shooter speed down
    btnJS3.whenReleased(() -> {});

    btnJS4.whenPressed(() -> {});
    btnJS4.whenReleased(() -> {});

    btnJS5.whenPressed(() -> {});  //manual shooter speed up
    btnJS5.whenReleased(() -> {}); 

    btnJS6.whenPressed(() -> intake.intakeIn()); 
    btnJS6.whenReleased(() -> intake.intakeStop());

    btnJS7.whenPressed(() -> intake.intakeOut()); 
    btnJS7.whenReleased(() -> intake.intakeStop());

    btnJS8.whenPressed(() -> hood.manualMoveHoodDown()); //add to megashooter once we can test that it works
    btnJS8.whenReleased(() -> hood.manualStopHoodMovement());

    btnJS9.whenPressed(() -> turret.turnTurret(-0.5)); //add to megashooter
    btnJS9.whenReleased(() -> turret.turnTurret(0));
    
    btnJS10.whenPressed(() -> turret.turnTurret(0.5));
    btnJS10.whenReleased(() -> turret.turnTurret(0));

    btnJS11.whenPressed(() -> elevator.manualDown());
    btnJS11.whenReleased(() -> elevator.manualStop());

    btnJS12.whenPressed(() -> elevator.manualUp());
    btnJS12.whenReleased(() -> elevator.manualStop());

    if(secondaryPanel.getY() < -.25) {
      elevator.setOverride(true);
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    final ShuffleboardTab tab = Shuffleboard.getTab("manageAuto");
    final NetworkTableEntry autoDelay = tab.add("Auto Delay", 0).getEntry();

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

    switch(AutoModeSelector.getSelectedAuto()) { 
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
