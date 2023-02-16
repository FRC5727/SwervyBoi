// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.omegabytes.library.OmegaLib.ControllerTypeBeat.JoystickButton;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.Autos.Auto;
import frc.robot.commands.Autos.ChargeStationRedSideAuto;
import frc.robot.commands.Autos.StraightLineAuto1;
import frc.robot.commands.Autos.StraightLineRedToCargo4Auto;
import frc.robot.commands.Songs.GiornosTheme;
import frc.robot.commands.Songs.ItsBeenSoLong;
import frc.robot.commands.Songs.Megalovania;
import frc.robot.commands.Songs.MichaelHunterThemeFromSanAndreas;
import frc.robot.commands.Songs.SwedenC418;
import frc.robot.commands.Songs.bohemianRhapsody;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;

import frc.robot.subsystems.IntakeSubsystem;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  //Subsystems
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();


  //Commands
  private final DriveCommand driveCommand = new DriveCommand(driveSubsystem);
  private final ArmCommand armCommand = new ArmCommand(armSubsystem);
  private final IntakeCommand intakeCommand = new IntakeCommand(intakeSubsystem);
  
  SendableChooser<Command> chooser = new SendableChooser<>();
  //Auto Routines 
  private final Auto auto = new Auto(driveSubsystem);
  private final StraightLineAuto1 straightLineAuto1 = new StraightLineAuto1(driveSubsystem);
  private final StraightLineRedToCargo4Auto straightLineRedToCargo4Auto = new StraightLineRedToCargo4Auto(driveSubsystem);
  private final ChargeStationRedSideAuto chargeStationRedSideAuto = new ChargeStationRedSideAuto(driveSubsystem);

  //Songs
  private final ItsBeenSoLong itsBeenSoLong = new ItsBeenSoLong(driveSubsystem);
  private final GiornosTheme giornosTheme = new GiornosTheme(driveSubsystem);
  private final SwedenC418 swedenC418 = new SwedenC418(driveSubsystem);
  private final MichaelHunterThemeFromSanAndreas michaelHunterThemeFromSanAndreas = new MichaelHunterThemeFromSanAndreas(driveSubsystem);
  private final Megalovania megalovania = new Megalovania(driveSubsystem);
  private final bohemianRhapsody bohemianRhapsody = new bohemianRhapsody(driveSubsystem);

  //Driver Buttons
  private final JoystickButton lowButton = new JoystickButton(Constants.dXboxController, Constants.aXboxButton);
  private final JoystickButton midButton = new JoystickButton(Constants.dXboxController, Constants.bXboxButton);
  private final JoystickButton highButton = new JoystickButton(Constants.dXboxController, Constants.xXboxButton);
  private final JoystickButton groundButton = new JoystickButton(Constants.dXboxController, Constants.yXboxButton);
  // private final JoystickButton chassisPosition = new JoystickButton(Constants.dXboxController, Constants.dXboxController.getPOV(Constants.povDown)); //180
  // private final JoystickButton lowPosition = new JoystickButton(Constants.dXboxController, Constants.dXboxController.getPOV(Constants.povLeft)); //270
  // private final JoystickButton midPosition = new JoystickButton(Constants.dXboxController, Constants.dXboxController.getPOV(Constants.povUp)); //0
  // private final JoystickButton highPosition = new JoystickButton(Constants.dXboxController, Constants.dXboxController.getPOV(Constants.povRight)); //90
  private final JoystickButton chassis = new JoystickButton(Constants.dXboxController, 8);
  private final JoystickButton intakeGroundPosition = new JoystickButton(Constants.dXboxController, Constants.lbXboxBumper);
  private final JoystickButton stationPickupPosition = new JoystickButton(Constants.dXboxController, Constants.rbXboxBumper);
  private final JoystickButton halfSpeed = new JoystickButton(Constants.dXboxController, Constants.backXboxButton);
  //Manip buttons 
  private final JoystickButton zeroGyroscope = new JoystickButton(Constants.mXboxController, Constants.backXboxButton);

  
	public RobotContainer() {
    driveSubsystem.setDefaultCommand(driveCommand);
    armSubsystem.setDefaultCommand(armCommand);
    intakeSubsystem.setDefaultCommand(intakeCommand);
    configureBindings();
    //Auto Routines
    chooser.setDefaultOption("Go forward and come back", auto);
    chooser.addOption("RED SIDE: Straight Line To Cargo 1 Auto", straightLineAuto1);
    chooser.addOption("RED SIDE: Straight Line To Cargo 4 Auto", straightLineRedToCargo4Auto);
    chooser.addOption("RED SIDE: Charge Station Auto", chargeStationRedSideAuto);

    //Songs
    chooser.addOption("It's Been So Long by The Living Tombstone", itsBeenSoLong);
    chooser.addOption("Ginornos Theme", giornosTheme);
    chooser.addOption("Sweden by C418", swedenC418);
    chooser.addOption("Michael Hunter Theme From San Andreas", michaelHunterThemeFromSanAndreas);
    chooser.addOption("Megalovania", megalovania);
    chooser.addOption("Bohemian Rhapsody by Queen", bohemianRhapsody);
    SmartDashboard.putData(chooser);

  }
  public Command getAutonomousCommand() {
    return chooser.getSelected();
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

    // chassis.onTrue(new InstantCommand(() -> armCommand.chassisPos()));
    // // chassisPosition.onTrue(new InstantCommand(() -> armCommand.chassisPos()));
    // lowButton.onTrue(new InstantCommand(() -> armSubsystem.lowPos()));
    // midButton.onTrue(new InstantCommand(() -> armSubsystem.midPos()));
    // highButton.onTrue(new InstantCommand(() -> armSubsystem.highPos()));
    // groundButton.onTrue(new InstantCommand(() -> armSubsystem.intakeGroundPos()));

    // // stationPickupPosition.onTrue(new InstantCommand(() -> armCommand.stationPickupPos()));
    new JoystickButton(Constants.dXboxController, 1).whenPressed(() -> armSubsystem.highPos(), armSubsystem); //1=A
    new JoystickButton(Constants.dXboxController, 2).whenPressed(() -> armSubsystem.midPos(), armSubsystem); //2=B
    new JoystickButton(Constants.dXboxController, 3).whenPressed(() -> armSubsystem.lowPos(), armSubsystem); //3=X
    new JoystickButton(Constants.dXboxController, 4).whenPressed(() -> armSubsystem.intakeGroundPos(), armSubsystem); //4=Y
    new JoystickButton(Constants.dXboxController, 5).whenPressed(() -> armSubsystem.chassisPos(), armSubsystem); //5=LB
    //new JoystickButton(Constants.dXboxController, 6).whenReleased(() ->armSubsystem.intakeGroundPos(), armSubsystem); //6=RB
    //halfSpeed.onTrue(new InstantCommand(() -> driveSubsystem.toggleHalfSpeed()));

    /* MANIP BINDS */
    //zeroGyroscope.onTrue(new InstantCommand(() -> driveSubsystem.zeroGyroscope()));
    
  }
  public void updateAngle() {
    driveSubsystem.updateAngle();
  }
}

