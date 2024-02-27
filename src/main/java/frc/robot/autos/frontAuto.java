
package frc.robot.autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ToolSubsystem;

import java.util.List;

public class frontAuto extends SequentialCommandGroup {

  public frontAuto(SwerveSubsystem m_SwerveSubsystem, ToolSubsystem tools) {

    TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.SwerveConstants.swerveKinematics);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these  interior waypoints
            List.of(new Translation2d(0.5, 0)), 
            // End 1.5 meters straight ahead of where we started, facing forward
            new Pose2d(1, 0, new Rotation2d(0)),
            config);

    var thetaController =
        new ProfiledPIDController(
            Constants.AutoConstants.kPThetaController,
            0,
            0,
            Constants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    Trajectory exampleTrajectory2 =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(1, 0, new Rotation2d(0)),
            // Pass through these  interior waypoints
            List.of(new Translation2d(0.5, 0)), 
            // End 1.5 meters straight ahead of where we started, facing forward
            new Pose2d(0, 0, new Rotation2d(0)),
            config);
Trajectory exampleTrajectory3 =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these  interior waypoints
            List.of(new Translation2d(0.5, 0.5)), 
            // End 1.5 meters straight ahead of where we started, facing forward
            new Pose2d(1,1, new Rotation2d(0)),
            config);
    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            exampleTrajectory,
            m_SwerveSubsystem::getPose,
            Constants.SwerveConstants.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            m_SwerveSubsystem::setModuleStates,
            m_SwerveSubsystem);

    SwerveControllerCommand swerveControllerCommand2 =
        new SwerveControllerCommand(
            exampleTrajectory2,
            m_SwerveSubsystem::getPose,
            Constants.SwerveConstants.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            m_SwerveSubsystem::setModuleStates,
            m_SwerveSubsystem);

    SwerveControllerCommand swerveControllerCommand3 =
        new SwerveControllerCommand(
            exampleTrajectory3,
            m_SwerveSubsystem::getPose,
            Constants.SwerveConstants.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            m_SwerveSubsystem::setModuleStates,
            m_SwerveSubsystem);

    addCommands(
        new InstantCommand(() -> m_SwerveSubsystem.resetOdometry(exampleTrajectory.getInitialPose())),
        new InstantCommand(() -> tools.shooterToggleStart()),
        new InstantCommand(() -> tools.intakeStart()),
        swerveControllerCommand,
        new WaitCommand(10),
        new InstantCommand(() -> tools.shooterToggleStop()),
        new InstantCommand(() -> tools.intakeStop()) ,
        swerveControllerCommand2,
        new WaitCommand(10),
        new InstantCommand(() -> tools.shooterToggleStart()),
        new WaitCommand(2),
        swerveControllerCommand3,
        new WaitCommand(10),
        new InstantCommand(() -> tools.shooterToggleStop()),
        new InstantCommand(() -> tools.intakeStop())
        );

  }
}