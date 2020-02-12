/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class SmartIntakeCommand extends CommandBase {
  private IntakeSubsystem m_intakeSubsystem;
  private ConveyorSubsystem m_conveyorSubsystem;
  private DigitalInput ballSensor;
  /**
   * Creates a new SmartIntakeCommand.
   */
  public SmartIntakeCommand(ConveyorSubsystem conveyorSubsystem, IntakeSubsystem intakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    ballSensor = new DigitalInput(Constants.ConveyorSubsystem.kBallSensorID);
    m_conveyorSubsystem = conveyorSubsystem;
    m_intakeSubsystem = intakeSubsystem;
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    while (!ballSensor.get()) {
      m_intakeSubsystem.intakeIn();
      m_conveyorSubsystem.preLoad();
    } 
    m_conveyorSubsystem.lifterStop();
    m_intakeSubsystem.intakeIn();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.intakeStop();
    m_conveyorSubsystem.lifterStop();  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
