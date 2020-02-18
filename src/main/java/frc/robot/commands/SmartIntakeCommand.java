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
  private DigitalInput middleBallSensor;
  private DigitalInput frontBallSensor;
  private DigitalInput topBallSensor;
  /**
   * Creates a new SmartIntakeCommand.
   */
  public SmartIntakeCommand(ConveyorSubsystem conveyorSubsystem, IntakeSubsystem intakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    middleBallSensor = new DigitalInput(Constants.ConveyorSubsystem.kMiddleBallSensorID);
    frontBallSensor = new DigitalInput(Constants.ConveyorSubsystem.kFrontBallSensorID);
    topBallSensor = new DigitalInput(Constants.ConveyorSubsystem.kTopBallSensorID);
    m_conveyorSubsystem = conveyorSubsystem;
    m_intakeSubsystem = intakeSubsystem;
    addRequirements(intakeSubsystem);
  }

  public boolean seeBall(DigitalInput sensor){
    return !sensor.get();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!seeBall(topBallSensor)){
      if (!seeBall(middleBallSensor)) {
        m_conveyorSubsystem.setWantPreload(true);
      } else if (!seeBall(frontBallSensor) && seeBall(middleBallSensor)){
        m_conveyorSubsystem.setWantPreload(false);
      } else if (seeBall(frontBallSensor) && seeBall(middleBallSensor)){
        m_conveyorSubsystem.setWantPreload(true);
      } else {
        m_conveyorSubsystem.setWantPreload(false);
      }
    } else {
      m_conveyorSubsystem.setWantPreload(false);
      System.out.println("LOADING");
    } else {
      m_conveyorSubsystem.setWantPreload(true);
      System.out.println("NOT PRELOADING");
    }
    m_intakeSubsystem.intakeIn();
    //System.out.println("ballSensor " + middleBallSensor.get());
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.intakeStop();
    m_conveyorSubsystem.setWantPreload(false);  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
