// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase {
  private final DriveSubsystem drive;
  
  private SlewRateLimiter translationXLimiter;
  private SlewRateLimiter translationYLimiter;
  private SlewRateLimiter rotationLimiter;

  private double translationXPercent;
  private double translationYPercent;
  private double rotationPercent;

  private int gearShift = 1;

  public DriveCommand(DriveSubsystem drive) {
    this.drive = drive;
    translationXLimiter = new SlewRateLimiter(Constants.translationRateLimit);
    translationYLimiter = new SlewRateLimiter(Constants.translationRateLimit);
    rotationLimiter = new SlewRateLimiter(Constants.rotationRateLimit);
    addRequirements(drive);
  }
  @Override
  public void initialize() {}

  @Override
  public void execute() {
    translationXPercent = -Constants.dXboxController.getRawAxis(1);
    translationYPercent = -Constants.dXboxController.getRawAxis(0);
    rotationPercent = Constants.dXboxController.getRawAxis(4);
    
    translationXPercent = translationXLimiter.calculate(translationXPercent);
    translationYPercent = translationYLimiter.calculate(translationYPercent);
    rotationPercent = rotationLimiter.calculate(rotationPercent);
    
    gearShift = Constants.dXboxController.getRawButtonReleased(Constants.bXboxButton) && gearShift < 5 ? gearShift + 1 : Constants.dXboxController.getRawButtonReleased(Constants.xXboxButton) && gearShift > 1 ? gearShift - 1 : gearShift;
    double multiplicationValue = Constants.dXboxController.getBackButtonReleased() ? gearShift * 0.2 : Constants.speedLimit;
    translationXPercent *= multiplicationValue;
    translationYPercent *= multiplicationValue;
    rotationPercent *= Constants.dXboxController.getBackButtonReleased() ? multiplicationValue : Constants.rSpeedLimit;

    if (Math.abs(translationXPercent) < Constants.deadzone){
      translationXPercent = 0.0;
    }
    if (Math.abs(translationYPercent) < Constants.deadzone){
      translationYPercent = 0.0;
    }
    if (Math.abs(rotationPercent) < Constants.deadzone){
      rotationPercent = 0.0;
    }
  } 
  @Override
  public void end(boolean interrupted) {
    drive.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }
  @Override
  public boolean isFinished() {
    return false;
  }
}
