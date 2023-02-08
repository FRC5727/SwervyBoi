// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

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
  
  public ArmSubsystem() {
    lowerArmSlave.follow(lowerArmMaster);
    lowerArmSlave.setInverted(InvertType.OpposeMaster);
  }

  public void lowerArmBrake(){
    lowerArmMaster.setNeutralMode(NeutralMode.Brake);
  }

  public void lowerArmCoast(){
    lowerArmMaster.setNeutralMode(NeutralMode.Coast);
  }

  public void lowerArmLowPosition(){
    if(lowerPos < 80){
      //lowerArmSlave.follow(lowerArmMaster);
      lowerArmMaster.set(ControlMode.PercentOutput, 0.25);
      //lowerArmSlave.setInverted(InvertType.OpposeMaster);
    }
    if(lowerPos > 80 &&  lowerPos < 90){
      lowerArmBrake();
    }
  }

  public void lowerArmMidPosition(){

  }

  public void lowerArmHighPosition(){

  }

  public void highArmLowPosition(){

  }

  public void highArmMidPosition(){

  }

  public void highArmHighPosition(){

  }
  @Override
  public void periodic() {
    lowerPos = lowerArmCanCoder.getPosition();
  }
}
