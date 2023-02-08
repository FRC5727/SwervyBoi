// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class ManualArmCommand extends CommandBase {
  /** Creates a new ManualArmCommand. */
  private final CANCoder upperArmCanCoder = new CANCoder(0, getName());
  private final CANCoder lowerArmCanCoder = new CANCoder(0, getName());

  private final TalonFX upperArmMaster = new TalonFX(0, getName());
  private final TalonFX upperArmSlave = new TalonFX(0, getName());

  private final TalonFX lowerArmMaster = new TalonFX(0, getName());
  private final TalonFX lowerArmSlave = new TalonFX(0, getName());


  public ManualArmCommand() {
    if(Constants.mXboxController.getBackButtonReleased()){

      double armSpeed = 0.25;
      double upperArmControl = Constants.mXboxController.getLeftY() * armSpeed;
      double lowerArmControl = Constants.mXboxController.getRightY() * armSpeed;

      upperArmSlave.follow(upperArmMaster);
      lowerArmSlave.follow(lowerArmMaster);

      upperArmMaster.set(ControlMode.PercentOutput, upperArmControl);
      lowerArmMaster.set(ControlMode.PercentOutput, lowerArmControl);
      
      upperArmSlave.setInverted(InvertType.OpposeMaster);
      lowerArmSlave.setInverted(InvertType.OpposeMaster);
    }
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
