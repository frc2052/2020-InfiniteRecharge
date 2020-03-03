/*----------------------------------------------------------------------------*/

/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */

/* Open Source Software - may be modified and shared by FRC teams. The code   */

/* must be accompanied by the FIRST BSD license file in the root directory of */

/* the project.                                                               */

/*----------------------------------------------------------------------------*/



package frc.robot.commands;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
import frc.robot.subsystems.PixyCamSubsystem;
import frc.robot.subsystems.PixyCamSubsystem.PixyBlock;
import frc.robot.Constants;

public class PixyCamManualDriveCommand extends CommandBase {
  private PixyCamSubsystem pixyCam = new PixyCamSubsystem();
  private Constants constants = new Constants();
  private DriveTrainSubsystem driveTrain;
  private Joystick tankJoy;

  /*
   * Creates a new PixyCamDrive.
   */

  public PixyCamManualDriveCommand(DriveTrainSubsystem drive, Joystick joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrain = drive;
    tankJoy = joystick;
    addRequirements(driveTrain);
  }

  public void getBallPosition() { 
    double xOffset;

    ArrayList<PixyBlock> list = pixyCam.read();

    if (list != null && list.size() > 0){
      Collections.sort(list, new Comparator<PixyBlock>() {
      @Override
      public int compare(PixyBlock first, PixyBlock second) {
        return first.height - second.height;
      }

      });

      Collections.reverse(list);
    } else {
      System.out.println("incoming list empty");
    }

    if(list !=null && list.size() > 0){
      PixyBlock biggestBall = list.get(0);
       xOffset = biggestBall.centerX - 155;
      if (xOffset > 100){
        xOffset = 100;
      } else if (xOffset < -100){
        xOffset = -100;
      }

      double turnPercent = -xOffset/100.0;
      double turnSpeed = Constants.PixyCamDriveConstants.turnSpeed * turnPercent;
      driveTrain.arcadeDrive(-tankJoy.getY(), turnSpeed);
    } else {
      driveTrain.arcadeDrive(0, 0);
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    getBallPosition();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}