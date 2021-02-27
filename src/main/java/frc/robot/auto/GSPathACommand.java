/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.auto.TrajectoryFactory.DrivePathEnum;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.PixyCamSubsystem.PixyBlock;
public class GSPathACommand extends SequentialCommandGroup {

    PixyCamSubsystem pixyCam = null;

    public TrajectoryFactory trajectoryFactory = new TrajectoryFactory();

    public GSPathACommand(DriveTrainSubsystem driveTrain, IntakeSubsystem intake, VisionSubsystem vision) {
        // Command path1 = trajectoryFactory.getRamseteCommand(driveTrain, DrivePathEnum.GalacticSearchABlue1);
        // this.addCommands(new ShiftGearCommand(driveTrain, true));
        // this.addCommands(path1);
        // this.addCommands(new ShiftGearCommand(driveTrain, false));
        // Command path2 = trajectoryFactory.getRamseteCommand(driveTrain, DrivePathEnum.GalacticSearchARed2);
        // ArmDownCommand intakeStart = new ArmDownCommand(intake);
        // ParallelCommandGroup driveIntake = new ParallelCommandGroup(path1, intakeStart);
        // this.addCommands(driveIntake);
        // this.addCommands(new ArmUpCommand(intake));
        pixyCam = new PixyCamSubsystem();

        ArrayList<PixyBlock> ballArray = pixyCam.read();
        ballArray = pixyCam.sortListByX(ballArray);

        if(ballArray.size() == 3){
            PixyBlock firstBall = ballArray.get(0);
            PixyBlock secondBall = ballArray.get(1);
            PixyBlock thirdBall = ballArray.get(2);
            //red path
            if(firstBall.centerX >=190 && (secondBall.centerX >=100 && secondBall.centerX<=140)){
                System.out.println("Path A Red");
                Command path1 = trajectoryFactory.getRamseteCommand(driveTrain, DrivePathEnum.GalacticSearchARed1);
                Command path2 = trajectoryFactory.getRamseteCommand(driveTrain, DrivePathEnum.GalacticSearchARed2);
                this.addCommands(path1);
                this.addCommands(path2);
                
            } 
            //blue path
            else{
                System.out.println("Path A Blue");
                Command path1 = trajectoryFactory.getRamseteCommand(driveTrain, DrivePathEnum.GalacticSearchABlue1);
                Command path2 = trajectoryFactory.getRamseteCommand(driveTrain, DrivePathEnum.GalacticSearchABlue2);
                this.addCommands(path1);
                this.addCommands(path2);
            }
            
        } else{
            System.out.println("ball array is not 3");
        }
        
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
  }
}
