// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// import frc.omegabytes.library.OmegaLib.ControllerTypeBeat.JoystickButton;
import frc.robot.commands.*;
import frc.robot.commands.Autos.*;
// import frc.robot.commands.Songs.*;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.Position;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
//  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final Swerve s_Swerve = new Swerve();

  private Position driverTargetPosition = Position.CHASSIS;

  // Commands
  // private final DriveCommand driveCommand = new DriveCommand(driveSubsystem);
  private final ArmCommand armCommand = new ArmCommand(armSubsystem);
  private final IntakeCommand intakeCommand = new IntakeCommand(intakeSubsystem);
  
  SendableChooser<Command> chooser = new SendableChooser<>();
  // Auto Routines 
  // private final Auto auto = new Auto(s_Swerve);
  // private final StraightLineAuto1 straightLineAuto1 = new StraightLineAuto1(s_Swerve);
  // private final StraightLineRedToCargo4Auto straightLineRedToCargo4Auto = new StraightLineRedToCargo4Auto(s_Swerve);
  private final ChargeStationRedSideAuto chargeStationRedSideAuto = new ChargeStationRedSideAuto(s_Swerve);
  private final RED2CubeAutoLeft red2CubeAutoLeft = new RED2CubeAutoLeft(s_Swerve);
  private final DoNothin doNothin = new DoNothin(s_Swerve);
  private final ChargeStationRedMobility chargeStationRedMobility = new ChargeStationRedMobility(s_Swerve);
  //Songs
  // private final ItsBeenSoLong itsBeenSoLong = new ItsBeenSoLong(driveSubsystem);
  // private final GiornosTheme giornosTheme = new GiornosTheme(driveSubsystem);
  // private final SwedenC418 swedenC418 = new SwedenC418(driveSubsystem);
  // private final MichaelHunterThemeFromSanAndreas michaelHunterThemeFromSanAndreas = new MichaelHunterThemeFromSanAndreas(driveSubsystem);
  // private final Megalovania megalovania = new Megalovania(driveSubsystem);
  // private final bohemianRhapsody bohemianRhapsody = new bohemianRhapsody(driveSubsystem);

  // Drive Controls
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  // Driver Buttons
  // private final JoystickButton robotCentric = new JoystickButton(Constants.dXboxController, XboxController.Button.kLeftBumper.value);

  // Manip buttons 
  private final JoystickButton zeroGyro = new JoystickButton(Constants.mXboxController, XboxController.Button.kBack.value);
  
	public RobotContainer() {
    s_Swerve.setDefaultCommand(
      new TeleopSwerve(
        s_Swerve,
        () -> -Constants.dXboxController.getRawAxis(translationAxis),
        () -> -Constants.dXboxController.getRawAxis(strafeAxis),
        () -> -Constants.dXboxController.getRawAxis(rotationAxis),
        () -> false // robotCentric.getAsBoolean()
      )
    );
    // armSubsystem.setDefaultCommand(armCommand);
    // intakeSubsystem.setDefaultCommand(intakeCommand);
    configureBindings();
    //Auto Routines
    chooser.setDefaultOption("RED SIDE: Charge Station + Mobility", chargeStationRedMobility);
    chooser.addOption("RED SIDE: 2 Cube Auto Left (Untested)", red2CubeAutoLeft);
    chooser.addOption("Do Nothin", doNothin);
    chooser.addOption("RED SIDE: Charge Station Auto", chargeStationRedSideAuto);

    //Songs
    // chooser.addOption("It's Been So Long by The Living Tombstone", itsBeenSoLong);
    // chooser.addOption("Ginornos Theme", giornosTheme);
    // chooser.addOption("Sweden by C418", swedenC418);
    // chooser.addOption("Michael Hunter Theme From San Andreas", michaelHunterThemeFromSanAndreas);
    // chooser.addOption("Megalovania", megalovania);
    // chooser.addOption("Bohemian Rhapsody by Queen", bohemianRhapsody);
    SmartDashboard.putData(chooser);

  }
  public Command getAutonomousCommand() {
    // TODO Re-enable auto selection
    // return chooser.getSelected();
    return new ChargeStationRedSideAuto(s_Swerve);
  }
  /* 
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    /* DRIVER BINDS */
    // cubeMode.onTrue(new InstantCommand(() -> intakeCommand.setCube()));
    // coneMode.onTrue(new InstantCommand(() -> intakeCommand.setCone()));

    // intake.onTrue(new InstantCommand(() -> intakeCommand.intake()));
    // place.onTrue(new InstantCommand(() -> intakeCommand.place()));
    // // stationPickupPosition.onTrue(new InstantCommand(() -> armCommand.stationPickupPos()));
    // new JoystickButton(Constants.dXboxController, XboxController.Axis.kLeftTrigger.value).whileTrue(intakeCommand);
    // new JoystickButton(Constants.dXboxController, XboxController.Axis.kRightTrigger.value).whileTrue(intakeCommand);

    // JoystickButton button = new JoystickButton(Constants.dXboxController, XboxController.Axis.kLeftTrigger.value);
    // JoystickButton button1 = new JoystickButton(Constants.dXboxController, XboxController.Axis.kRightTrigger.value);
    // button.and(button1).whileFalse(intakeCommand);

    // Driver arm controls
    // TODO Re-enable arm controls
    // new JoystickButton(Constants.dXboxController, XboxController.Button.kA.value).onTrue(Commands.runOnce(() -> driverTargetPosition = Position.GRID_LOW));
    // new JoystickButton(Constants.dXboxController, XboxController.Button.kB.value).onTrue(Commands.runOnce(() -> driverTargetPosition = Position.GRID_MID));
    // new JoystickButton(Constants.dXboxController, XboxController.Button.kY.value).onTrue(Commands.runOnce(() -> driverTargetPosition = Position.GRID_HIGH));
    // // new JoystickButton(Constants.dXboxController, XboxController.Button.kX.value).onTrue(Commands.runOnce(() -> driverTargetPosition = Position.CHASSIS));
    // new JoystickButton(Constants.dXboxController, XboxController.Button.kRightBumper.value)
    //   .whileTrue(
    //     Commands.runOnce(() -> armSubsystem.setTargetPosition(driverTargetPosition))
    //     .andThen(new ArmCommand(armSubsystem)))
    //   .onFalse(
    //     Commands.runOnce(() -> armSubsystem.setTargetPosition(Position.CHASSIS))
    //     .andThen(new ArmCommand(armSubsystem)));

    // TODO Move the TriggerButton, and make the intakeCommand actually work (runs forever, stops when command terminates, nice to have if finished when piece acquired)
    // new JoystickButton(Constants.dXboxController, XboxController.Button.kLeftBumper.value)
    // .whileTrue(
    //   Commands.runOnce(() -> armSubsystem.setTargetPosition(Position.INTAKE_SUBSTATION))
    //   .andThen(new ArmCommand(armSubsystem)))
    // .onFalse(
    //   Commands.runOnce(() -> Commands.runOnce(() -> armSubsystem.setTargetPosition(Position.CHASSIS)))
    //   .andThen(new ArmCommand(armSubsystem)));

    // new JoystickButton(Constants.dXboxController, XboxController.Button.kX.value).onTrue(Commands.runOnce(() -> intakeSubsystem.toggleCube()));

    //halfSpeed.onTrue(new InstantCommand(() -> driveSubsystem.toggleHalfSpeed()));

    /* Manip Buttons */
    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
  }

  // public void updateAngle() {
  //   driveSubsystem.updateAngle();
  // }
}