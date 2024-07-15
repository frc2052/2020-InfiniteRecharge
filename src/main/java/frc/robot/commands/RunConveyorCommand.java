/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConveyorSubsystem;

public class RunConveyorCommand extends Command {
  private final ConveyorSubsystem conveyor;

  public RunConveyorCommand(ConveyorSubsystem conveyor) {
    this.conveyor = conveyor;
    
    addRequirements(conveyor);
  }

  @Override
  public void execute() {
    conveyor.setWantUp(true);
  }

  @Override
  public void end(boolean interrupted) {
    conveyor.setWantUp(false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
