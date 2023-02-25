package frc.robot.commands.Autos;

import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;


public class exampleAuto extends SequentialCommandGroup {
    public exampleAuto(DriveSubsystem drive){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    1.0,
                    1.0)
                .setKinematics(drive.getKinematics());

        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1, 0)), //   new Translation2d(1, 1), new Translation2d(2, -1)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(2, 0, new Rotation2d(0)),
                config);

        var thetaController =
            new ProfiledPIDController(
                1, 0, 0, new TrapezoidProfile.Constraints(
                    Math.PI, Math.PI));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                exampleTrajectory,
                drive::getPose,
                drive.getKinematics(),
                new PIDController(1, 0, 0),
                new PIDController(1, 0, 0),
                thetaController,
                drive::setModuleStates,
                drive);


        // addCommands(
        //     new InstantCommand(() -> drive.resetOdometry(exampleTrajectory.getInitialPose())),
        //     swerveControllerCommand
        // );
    }
}
