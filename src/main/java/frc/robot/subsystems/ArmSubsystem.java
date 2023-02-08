// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  private final CANCoder upperArmCanCoder = new CANCoder(0, getName());
  private final CANCoder lowerArmCanCoder = new CANCoder(0, getName());

  private final TalonFX upperArmMaster = new TalonFX(0, getName());
  private final TalonFX upperArmSlave = new TalonFX(0, getName());

  private final TalonFX lowerArmMaster = new TalonFX(0, getName());
  private final TalonFX lowerArmSlave = new TalonFX(0, getName());

  private double lowerPos;

  ArmFeedforward feedforward = new ArmFeedforward(0, 0, 0, 0);

  private boolean lowerArmBrakeBool;
  private boolean highArmBrakeBool;

  public ArmSubsystem() {
    lowerArmSlave.follow(lowerArmMaster);
    lowerArmSlave.setInverted(InvertType.OpposeMaster);

    upperArmSlave.follow(upperArmMaster);
    upperArmSlave.setInverted(InvertType.OpposeMaster);
    lowerPos = lowerArmCanCoder.getPosition();
    //https://github.com/Mechanical-Advantage/RobotCode2023/blob/de23fe8c8bd5105c144f02f3d1b8e78f5d77a2e8/src/main/java/org/littletonrobotics/frc2023/subsystems/arm/Arm.java#L113-L118
    
  }

  
  public void lowerArmBrake(){
    lowerArmMaster.setNeutralMode(NeutralMode.Brake);
  }
  public void lowerArmCoast(){
    lowerArmMaster.setNeutralMode(NeutralMode.Coast);
  }
  public void highArmBrake(){
    upperArmMaster.setNeutralMode(NeutralMode.Brake);
  }
  public void highArmCoast(){
    upperArmMaster.setNeutralMode(NeutralMode.Coast);
  }

  public void lowerArmLowPositionStepOne(){
    if(lowerPos < 80){
      lowerArmMaster.set(ControlMode.PercentOutput, 0.25);
    }
    if(lowerPos > 80 &&  lowerPos < 90){
      lowerArmBrakeBool = true;
    }
  }
  public void lowerArmLowPositionStepTwo(){

  }
  public void highArmLowPosition(){

  }

  public void lowerArmMidPosition(){

  }

  public void lowerArmHighPosition(){

  }
  public void brakeCheck(){
    if(lowerArmBrakeBool){
      lowerArmBrake();
    } else {
      lowerArmCoast();
    }
    if(highArmBrakeBool){
      highArmBrake();
    } else {
      highArmCoast();
    }
  }
  public void armStartUp(){
    lowerArmBrakeBool = false;
    highArmBrakeBool = false;
    brakeCheck();
  }
  

  public void highArmMidPosition(){

  }

  public void highArmHighPosition(){

  }
  
  @Override
  public void periodic() {
    brakeCheck();
  }
}
