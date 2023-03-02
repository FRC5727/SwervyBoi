// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.EnumMap;
import java.util.Map;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  // Controls whether or not to update SmartDashboard
  private static final boolean armDebug = true;

  // Abstraction of the encode positions for a defined arm position
  private class ArmPosition {
    double lowerArmAngle;
    double upperArmAngle;

    public ArmPosition(double lower, double upper) {
      lowerArmAngle = lower;
      upperArmAngle = upper;
    }
  }

  // Enumeration of all defined arm positions
  public enum Position {
    STARTING,
    CHASSIS,
    SAFE,
    GRID_LOW,
    GRID_MID,
    GRID_HIGH,
    INTAKE_PREGROUND,
    INTAKE_GROUND,
    INTAKE_SUBSTATION,
  };

  // Next positions for the Arm to target
  private Deque<Position> targetPosition = new ArrayDeque<Position>();
  private Position lastPosition = Position.STARTING;

  // Lookup "table" for each defined arm position
  private final EnumMap<Position, ArmPosition> armPositions = new EnumMap<>(Map.of(
      // TODO Compare all positions to practice bot definitions
      Position.STARTING, new ArmPosition(-20, -53),
      Position.CHASSIS, new ArmPosition(-32, -38), // TODO Measure and define
      Position.SAFE, new ArmPosition(-32, 17),
      Position.GRID_LOW, new ArmPosition(-19, -33),
      Position.GRID_MID, new ArmPosition(1, -3), //lower arm is the one conncted directly to the chassis
      Position.GRID_HIGH, new ArmPosition(26, 24),
      Position.INTAKE_PREGROUND, new ArmPosition(-1, -58), // TODO Measure and define
      Position.INTAKE_GROUND, new ArmPosition(10, -68),
      Position.INTAKE_SUBSTATION, new ArmPosition(-8, 8)
    ));

  private WPI_TalonFX lowerArmMaster;
  private WPI_TalonFX lowerArmSlave;

  private WPI_TalonFX highArmMaster;
  private WPI_TalonFX highArmSlave;

  private CANCoder lowerArmCoder;
  private CANCoder highArmCoder;


  private double lowerArmGearRatio = 66.0 / 16.0;
  private double highArmGearRatio = 44.0 / 16.0;

  private PIDController lowPidController;
  private PIDController highPidController;

  private ArmFeedforward highArmFeedforward;
  private ArmFeedforward lowArmFeedforward;

  // TODO Tune this
  private double L_kp = 0.90,
      L_ki = 0.00,
      L_kd = 0.00;

  // TODO Tune this
  private double H_kp = 0.40,
      H_ki = 0.00,
      H_kd = 0.00;

  private double L_ks = 0.00,
      L_kg = 0.16, // 0.16
      L_kv = 8.08, // 8.08
      L_ka = 0.03; // 0.03

  private double H_ks = 0.00,
      H_kg = 0.21, // 0.21
      H_kv = 4.45, // 4.45
      H_ka = 0.02; // 0.02

  private double L_maxVoltage = 3;
  private double H_maxVoltage = 3;

  private double LowerCANcoderInitTime = 0.0;
  private double HighCANcoderInitTime = 0.0;

  public ArmSubsystem() {
    // TODO Get the device numbers from centrally defined constants
    this.lowerArmMaster = new WPI_TalonFX(9, Constants.CANivoreName);
    this.lowerArmSlave = new WPI_TalonFX(11, Constants.CANivoreName);
    this.lowerArmCoder = new CANCoder(Constants.lowCoder, Constants.CANivoreName);

    // TODO Get the device numbers from centrally defined constants
    this.highArmMaster = new WPI_TalonFX(8, Constants.CANivoreName);
    this.highArmSlave = new WPI_TalonFX(10, Constants.CANivoreName);
    this.highArmCoder = new CANCoder(Constants.highCoder, Constants.CANivoreName);

    this.lowPidController = new PIDController(L_kp, L_ki, L_kd);
    this.highPidController = new PIDController(H_kp, H_ki, H_kd);

    // Jimmy, why are we overriding this?  This is confusing, and I don't know why it's necessary
    this.lowerArmGearRatio = 0.40; // 4.125
    this.highArmGearRatio = 0.10;

    // TODO Define tolerance elsewhere
    lowPidController.disableContinuousInput();
    lowPidController.setTolerance(2, .5);

    highPidController.disableContinuousInput();
    highPidController.setTolerance(2, .5);

    lowerArmSlave.follow(lowerArmMaster);
    lowerArmSlave.setInverted(true);
    highArmSlave.follow(highArmMaster);
    highArmSlave.setInverted(false);
    highArmMaster.setInverted(true);

    lowerArmMaster.setNeutralMode(NeutralMode.Brake);
    highArmMaster.setNeutralMode(NeutralMode.Brake);
  }

  // Limit a value (positive or negative)
  private double constrainValue(double value, double max) {
    return Math.signum(value) * Math.min(Math.abs(value), max);
  }
  //for demo bot (atm)
  public double getLowRelativeAngle() {
    return lowerArmCoder.getPosition() * (360.0 / (lowerArmGearRatio * 4096.0));
  }
  //for demo bot (atm)
  public double getHighRelativeAngle() {
    return highArmCoder.getPosition() * (360.0 / (highArmGearRatio * 4096.0));
  }
  //for comp bot 
  public double getLowAbsoluteAngle(){
    return lowerArmCoder.getAbsolutePosition();
  }
  //for comp bot
  public double getHighAbsoluteAngle(){
    return highArmCoder.getAbsolutePosition();
  }

  public void setTargetPosition(Position position) {
    targetPosition.clear();
    if (!Constants.armPositionDebugDirect && lastPosition != position) {
      // Determine first step based on last known position
      switch (lastPosition) {
        case STARTING:
        case CHASSIS:
          targetPosition.add(Position.CHASSIS);
          break;
        case GRID_LOW:
        case GRID_MID:
        case GRID_HIGH:
        case INTAKE_SUBSTATION:
        case SAFE:
          targetPosition.add(Position.SAFE);
          break;
        case INTAKE_PREGROUND:
        case INTAKE_GROUND:
          targetPosition.add(Position.INTAKE_PREGROUND);
          break;
      }

      // Now we know we are transitioning from either CHASSIS, SAFE, or INTAKE_PREGROUND
      // Determine any prequisites for final position
      switch (position) {
        case STARTING:
          if (targetPosition.isEmpty() || targetPosition.getLast() != Position.CHASSIS) {
            targetPosition.add(Position.CHASSIS);
          }
          break;
        case INTAKE_GROUND:
        case INTAKE_PREGROUND:
          if (targetPosition.isEmpty() || targetPosition.getLast() != Position.INTAKE_PREGROUND) {
            targetPosition.add(Position.GRID_LOW);
            targetPosition.add(Position.INTAKE_PREGROUND);
          }
          break;
        case GRID_MID:
        case GRID_HIGH:
        case INTAKE_SUBSTATION:
          if (targetPosition.isEmpty() || targetPosition.getLast() != Position.SAFE) {
            targetPosition.add(Position.SAFE);
          }
          break;
        case CHASSIS:
          if (targetPosition.isEmpty() || targetPosition.getLast() == Position.INTAKE_PREGROUND) {
            targetPosition.add(Position.GRID_LOW);
          }
          break;
        case GRID_LOW:
        case SAFE:
          // Safe directly from any of the previous positions
          break;
      }
    }
    if (targetPosition.isEmpty() || targetPosition.getLast() != position) {
      targetPosition.add(position);
    }
  }

  public void beginMovement() {
    ArmPosition nextPosition = armPositions.get(targetPosition.peek());

    lowPidController.setSetpoint(nextPosition.lowerArmAngle);
    lowPidController.reset();
    highPidController.setSetpoint(nextPosition.upperArmAngle);
    highPidController.reset();
  }

  public void updateMovement() {
    if (lowPidController.atSetpoint() && highPidController.atSetpoint()) {
      lastPosition = targetPosition.remove();
      if (!doneMovement()) {
        beginMovement();
      }
    }
    if (!doneMovement()) {
      lowerArmMaster.setVoltage(constrainValue(lowPidController.calculate(getLowAbsoluteAngle()), L_maxVoltage));
      highArmMaster.setVoltage(constrainValue(highPidController.calculate(getHighAbsoluteAngle()), H_maxVoltage));
    } else {
      stopMovement();
    }
  }

  public boolean doneMovement() {
    return targetPosition.isEmpty();
  }

  public void stopMovement() {
    lowerArmMaster.stopMotor();
    highArmMaster.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Update dashboard to help monitor and debug

    if (armDebug) {
      SmartDashboard.putNumber("Target queue depth", targetPosition.size());
      SmartDashboard.putString("Target Position", targetPosition.isEmpty() ? "<none>" : targetPosition.peek().toString());
      SmartDashboard.putNumber("Low Arm Setpoint: ", lowPidController.getSetpoint());
      SmartDashboard.putNumber("High Arm Setpoint: ", highPidController.getSetpoint());
      SmartDashboard.putBoolean("Low Arm set? ", lowPidController.atSetpoint());
      SmartDashboard.putBoolean("High Arm set? ", highPidController.atSetpoint());

      SmartDashboard.putNumber("Low Angle ABSOLUTE", lowerArmCoder.getAbsolutePosition());
      SmartDashboard.putNumber("High Angle ABSOLUTE", highArmCoder.getAbsolutePosition());

      SmartDashboard.putNumber("Low Arm Master Voltage:", lowerArmMaster.getMotorOutputVoltage());
      SmartDashboard.putNumber("Low Arm Slave Voltage:", lowerArmSlave.getMotorOutputVoltage());

      SmartDashboard.putNumber("High Arm Master Voltage:", highArmMaster.getMotorOutputVoltage());
      SmartDashboard.putNumber("High Arm Slave Voltage:", highArmSlave.getMotorOutputVoltage());
    }
  }
}