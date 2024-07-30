/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.*;
import frc.robot.commands.*;


/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private DriveTrainSubsystem drivetrain;
  private IntakeSubsystem intake;
  private ShooterSubsystem shooter;
  private HoodSubsystem hood;
  private TurretSubsystem turret;
  private ConveyorSubsystem conveyor;

  private Joystick joystick = new Joystick(0);

  public RobotContainer() {
    drivetrain = new DriveTrainSubsystem();
    intake = new IntakeSubsystem();
    shooter = new ShooterSubsystem();
    hood = new HoodSubsystem();
    turret = new TurretSubsystem();
    conveyor = new ConveyorSubsystem();

    drivetrain.setDefaultCommand(
      new RunCommand(
        () -> drivetrain.curvatureDrive(-joystick.getX(), -joystick.getY(), !joystick.getRawButton(3)), 
        drivetrain
      )
    );

    configureButtonBindings();
  }

  public void resetEncoders() {
    drivetrain.resetEncoders();
    hood.resetEncoder();
    turret.resetEncoder();
  }

  public void configureButtonBindings() {

    JoystickButton btnJS1  = new JoystickButton(joystick, 1);
    JoystickButton btnJS2  = new JoystickButton(joystick, 2);
    JoystickButton btnJS3  = new JoystickButton(joystick, 3);
    JoystickButton btnJS4  = new JoystickButton(joystick, 4);
    JoystickButton btnJS5  = new JoystickButton(joystick, 5);
    JoystickButton btnJS6  = new JoystickButton(joystick, 6);
    JoystickButton btnJS7  = new JoystickButton(joystick, 7);
    JoystickButton btnJS8  = new JoystickButton(joystick, 8);
    JoystickButton btnJS9  = new JoystickButton(joystick, 9);
    JoystickButton btnJS10 = new JoystickButton(joystick, 10);
    JoystickButton btnJS11 = new JoystickButton(joystick, 11);
    JoystickButton btnJS12 = new JoystickButton(joystick, 12);

    btnJS2.onTrue(new InstantCommand(() -> drivetrain.setHighGear(true))); //Shift speeds
    btnJS2.onFalse(new InstantCommand(() -> drivetrain.setHighGear(false))); //stop shifting

    btnJS1.onTrue(new IntakeCommand(intake, conveyor));
    
    btnJS4.whileTrue(new ThrottleShootCommand(shooter, joystick::getThrottle));
    
    btnJS6.whileTrue(new RunConveyorCommand(conveyor));

    btnJS7.onTrue(new InstantCommand(() -> hood.manualMoveHoodDown())); 
    btnJS7.onFalse(new InstantCommand(() -> hood.manualStopHoodMovement()));

    btnJS8.onTrue(new InstantCommand(() -> hood.manualMoveHoodUp()));
    btnJS8.onFalse(new InstantCommand(() -> hood.manualStopHoodMovement()));

    btnJS9.whileTrue(new TurretMoveCommand(0.3, turret));

    btnJS10.whileTrue(new TurretMoveCommand(-0.3, turret));

    btnJS11.onTrue(new InstantCommand(() -> intake.armDown()));
    btnJS12.onTrue(new InstantCommand(() -> intake.armUp()));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
