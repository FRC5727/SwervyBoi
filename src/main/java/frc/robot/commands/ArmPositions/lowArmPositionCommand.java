// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmPositions;

import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmSubsystem;

public class lowArmPositionCommand extends SequentialCommandGroup {
  /** Creates a new lowArmPositionCommand. */
  public lowArmPositionCommand(ArmSubsystem armSubsystem) {
    final CANCoder upperArmCanCoder = new CANCoder(0, getName());
    final CANCoder lowerArmCanCoder = new CANCoder(0, getName());
    addCommands(
      new InstantCommand(() -> armSubsystem.armStartUp()),
      new WaitCommand(0.5),
      new InstantCommand(() -> armSubsystem.lowerArmLowPositionStepOne()),
      new WaitCommand(5),
      new InstantCommand(() -> armSubsystem.highArmLowPosition()),
      new WaitCommand(5),
      new InstantCommand(() -> armSubsystem.lowerArmLowPositionStepTwo())
    );
  }
}
