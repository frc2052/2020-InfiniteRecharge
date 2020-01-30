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
import frc.robot.subsystems.HoodSubsystem.anglePresetEnum;
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


  private final DriveTrainSubsystem driveTrain = new DriveTrainSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final HoodSubsystem hood = new HoodSubsystem();
  private final TurretSubsystem turret = new TurretSubsystem();
  private final VisionSubsystem vision = new VisionSubsystem();
  private final ConveyorSubsystem conveyor = new ConveyorSubsystem();

  private final Joystick leftJoystick = new Joystick(0);
  private final Joystick rightJoystick = new Joystick(1);
  private final Joystick secondaryJoystick = new Joystick(2);

  private boolean manualShooterIncrease;
  private boolean manualShooterDecrease;
  private boolean manualHoodUp;
  private boolean manualHoodDown;
  private boolean manualTurretLeft;
  private boolean manualTurretRight;
  private boolean shootPressed;
  private boolean readyPressed;
  private boolean conveyorDownPressed;

  private final MegaShooterCommand megaShooterCommand  = new MegaShooterCommand(shooter, vision, hood, turret, conveyor, manualShooterIncrease, manualShooterDecrease, manualHoodUp, manualHoodDown, manualTurretLeft, manualTurretRight, shootPressed, readyPressed, conveyorDownPressed);

  private final CenterShootDriveParkCommand centerShootDrivePark = new CenterShootDriveParkCommand(driveTrain, shooter, vision, hood, turret, conveyor, autoDelay.getDouble(0));
  private final StartLeftGenerator3Command leftGenerator3 = new StartLeftGenerator3Command(driveTrain, shooter, intake, vision, hood, turret, conveyor, autoDelay.getDouble(0));
  private final StartLeftTrench2Command leftTrench2 = new StartLeftTrench2Command(driveTrain, shooter, intake, vision, hood, turret, conveyor, autoDelay.getDouble(0));
  private final StartRightTrench3Command rightTrench3 = new StartRightTrench3Command(driveTrain, shooter, intake, vision, hood, turret, conveyor, autoDelay.getDouble(0));



  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    driveTrain.setDefaultCommand(
      // Default to basic tank drive
      new RunCommand(
        () -> driveTrain.tankDrive(leftJoystick.getY(), rightJoystick.getY() ), 
        driveTrain
      )
    );


    shooter.setDefaultCommand(
      megaShooterCommand
    );
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //TODO : configure buttons for what drivers want
    //left joystick
    JoystickButton btnJL1 = new JoystickButton(leftJoystick, 1);
    JoystickButton btnJL2 = new JoystickButton(leftJoystick, 2);
    JoystickButton btnJL3 = new JoystickButton(leftJoystick, 3);
    JoystickButton btnJL4 = new JoystickButton(leftJoystick, 4);
    JoystickButton btnJL5 = new JoystickButton(leftJoystick, 5);
    JoystickButton btnJL6 = new JoystickButton(leftJoystick, 6);
    JoystickButton btnJL7 = new JoystickButton(leftJoystick, 7);
    JoystickButton btnJL8 = new JoystickButton(leftJoystick, 8);
    JoystickButton btnJL9 = new JoystickButton(leftJoystick, 9);
    JoystickButton btnJL10 = new JoystickButton(leftJoystick, 10);
    JoystickButton btnJL11 = new JoystickButton(leftJoystick, 11);
    JoystickButton btnJL12 = new JoystickButton(leftJoystick, 12);

    btnJL1.whenPressed(() -> {manualShooterIncrease = true;});
    btnJL2.whenPressed(() -> {readyPressed = true;}); 
    btnJL3.whenPressed(() -> {manualHoodUp = true;}); 
    btnJL4.whenPressed(() -> {manualHoodDown = true;}); 
    btnJL5.whenPressed(() -> {manualTurretLeft = true;}); 
    btnJL6.whenPressed(() -> {manualTurretRight = true;}); 
    btnJL7.whenPressed(() -> {manualShooterDecrease = true; }); 
    btnJL8.whenPressed(() -> {readyPressed = true;}); 
    btnJL9.whenPressed(() -> {conveyorDownPressed = true;}); 
    btnJL10.whenPressed(() -> {}); 
    btnJL11.whenPressed(() -> {}); 
    btnJL12.whenPressed(() -> {}); 

    btnJL1.whenReleased(() -> {manualShooterIncrease = false;});
    btnJL2.whenReleased(() -> {readyPressed = false;}); 
    btnJL3.whenReleased(() -> {manualHoodUp = false;}); 
    btnJL4.whenReleased(() -> {manualHoodDown = false;}); 
    btnJL5.whenReleased(() -> {manualTurretLeft = false;}); 
    btnJL6.whenReleased(() -> {manualTurretRight = false;}); 
    btnJL7.whenReleased(() -> {shootPressed = false;}); 
    btnJL8.whenReleased(() -> {manualShooterDecrease = false;}); 
    btnJL9.whenReleased(() -> {conveyorDownPressed = false;}); 

    //right joystick
    JoystickButton btnJR1 = new JoystickButton(rightJoystick, 1);
    JoystickButton btnJR2 = new JoystickButton(rightJoystick, 2);
    JoystickButton btnJR3 = new JoystickButton(rightJoystick, 3);
    JoystickButton btnJR4 = new JoystickButton(rightJoystick, 4);
    JoystickButton btnJR5 = new JoystickButton(rightJoystick, 5);
    JoystickButton btnJR6 = new JoystickButton(rightJoystick, 6);
    JoystickButton btnJR7 = new JoystickButton(rightJoystick, 7);
    JoystickButton btnJR8 = new JoystickButton(rightJoystick, 8);
    JoystickButton btnJR9 = new JoystickButton(rightJoystick, 9);
    JoystickButton btnJR10 = new JoystickButton(rightJoystick, 10);
    JoystickButton btnJR11 = new JoystickButton(rightJoystick, 11);
    JoystickButton btnJR12 = new JoystickButton(rightJoystick, 12);

    btnJR1.whenPressed(() -> shooter.setSpeed(Constants.Shooter.kShooterSpeedRPS));
    btnJR2.whenPressed(() -> driveTrain.setHighGear(true)); //Shift speeds
    btnJR2.whenReleased(() -> driveTrain.setHighGear(false)); //stop shifting
    btnJR3.whenPressed(() -> {}); 
    btnJR4.whenPressed(() -> {}); 
    btnJR5.whenPressed(() -> {}); 
    btnJR6.whenPressed(() -> {}); 
    btnJR7.whenPressed(() -> {}); 
    btnJR8.whenPressed(() -> {}); 
    btnJR9.whenPressed(() -> {}); 
    btnJR10.whenPressed(() -> {}); 
    btnJR11.whenPressed(() -> {}); 
    btnJR12.whenPressed(() -> {}); 

    //secondary control panel
    JoystickButton btnJSB1 = new JoystickButton(secondaryJoystick, 1);
    JoystickButton btnJSB2 = new JoystickButton(secondaryJoystick, 2);
    JoystickButton btnJSB3 = new JoystickButton(secondaryJoystick, 3);
    JoystickButton btnJSB4 = new JoystickButton(secondaryJoystick, 4);
    JoystickButton btnJSB5 = new JoystickButton(secondaryJoystick, 5);
    JoystickButton btnJSB6 = new JoystickButton(secondaryJoystick, 6);
    JoystickButton btnJSB7 = new JoystickButton(secondaryJoystick, 7);
    JoystickButton btnJSB8 = new JoystickButton(secondaryJoystick, 8);
    JoystickButton btnJSB9 = new JoystickButton(secondaryJoystick, 9);
    JoystickButton btnJSB10 = new JoystickButton(secondaryJoystick, 10);
    JoystickButton btnJSB11 = new JoystickButton(secondaryJoystick, 11);
    JoystickButton btnJSB12 = new JoystickButton(secondaryJoystick, 12);

    btnJSB1.whenPressed(() -> intake.armToggle());
    btnJSB2.whenPressed(() -> intake.intakeIn());
    btnJSB2.whenReleased(() -> intake.intakeStop());
    btnJSB3.whenPressed(() -> intake.intakeOut());
    btnJSB3.whenReleased(() -> intake.intakeStop());
    btnJSB4.whenPressed(() -> {});
    btnJSB5.whenPressed(() -> {}); 
    btnJSB6.whenPressed(() -> {}); 
    
    if(SmartDashboard.getBoolean("Shooter Override?", false) == true){
      btnJSB7.whenPressed(() -> shooter.setSpeed(Constants.Shooter.kShooterSpeedRPS)); 
    }

    if(SmartDashboard.getBoolean("Turret Override?", false) == true) {
      btnJSB8.whenPressed(() -> turret.turnTurret(Constants.Turret.kTurnLeftSpeed)); 
      btnJSB9.whenPressed(() -> turret.turnTurret(Constants.Turret.kTurnRightSpeed)); 
    }

    if(SmartDashboard.getBoolean("Hood Override?", false) == true) {
      btnJSB10.whenPressed(() -> hood.setTarget(anglePresetEnum.CLOSE)); 
      btnJSB11.whenPressed(() -> hood.setTarget(anglePresetEnum.MIDDLE)); 
      btnJSB12.whenPressed(() -> hood.setTarget(anglePresetEnum.FAR)); 
    }
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
        return leftGenerator3;
      case LSG5:
        return leftGenerator3; //not correct
      case LST2:
        return leftTrench2;
      case RST3:
        return rightTrench3;
      case CSG3:
        return centerShootDrivePark; //not correct
      case CS:
        return centerShootDrivePark;
      default:
        return centerShootDrivePark; //not correct, make don't move auto
    }
  }
}
