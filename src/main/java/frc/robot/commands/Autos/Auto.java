// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class Auto extends SequentialCommandGroup {
  /** Creates a new Auto. */
  public Auto(DriveSubsystem driveSubsystem) {
    Constants.translationXController.reset();
    Constants.translationYController.reset();
    Constants.rotationController.reset();

    PathPlannerTrajectory test1 = PathPlanner.loadPath("Charge2ElectricBoogaloo", 1.00, 1.00); 
    //install vendor
     
    /* 
    https://3015rangerrobotics.github.io/pathplannerlib/PathplannerLib.json 
    */
    
    addCommands(
      new InstantCommand(() -> driveSubsystem.zeroGyroscope()),
      new WaitCommand(1.0),
      new InstantCommand(() -> driveSubsystem.resetPose(2.12, 2.75)),
      new PPSwerveControllerCommand(
        test1,
        driveSubsystem::getPose,
        driveSubsystem.getKinematics(),
        new PIDController(1.0, 0.0, 0.2), //X
        new PIDController(0.75, 0, 0.2), //Y
        new PIDController(0.05, 0, 0.10), //Constants.rotationController
        driveSubsystem::setModuleStates,
        driveSubsystem
      ),
      new InstantCommand(() -> driveSubsystem.stop()),
      new InstantCommand(() -> driveSubsystem.endAuto()),
      new WaitCommand(0.5)
      );
  }

  
}
