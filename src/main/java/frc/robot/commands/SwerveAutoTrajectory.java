// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

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
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveAutoTrajectory extends SequentialCommandGroup {
    public SwerveAutoTrajectory(SwerveSubsystem swerveSubsystem){
        // Specifies how fast we want the robot to drive
        // (maximum speed, maximum acceleration, and kinematics of chassis)
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(DriveConstants.kDriveKinematics);
        
        // We need to determine the points the robot will drive to
        // Generates trajectory
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            // Defines the starting pose (position and orientation) of the robot
            new Pose2d(0, 0, new Rotation2d(0)),
            // Defines a list of interior waypoints (in meters) that the trajectory should pass through
            List.of(
                new Translation2d(0, 0),  // Waypoint 1
                new Translation2d(0, 0)   // Waypoint 2
            ),
            // Defines the ending pose (position and orientation) of the robot
            new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
            // Provides the trajectory configuration (maximum speed, maximum acceleration, and kinematics)
            trajectoryConfig
        );

        // PID controller to correct errors in the trajectory
        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        // PID controller for angle with constraints on max speed and acceleration (defined in Constants.java)
        ProfiledPIDController thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        // Since the robot's heading is from -180 to 180 degrees, we will make the angular controller continuous to handle that
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // Constructs a new SwerveControllerCommand that follows trajectory
        // Ctrl + click "SwerveControllerCommand" class for more information
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            // Generated trajectory (above in code)
            trajectory,
            // Gets robot coordinates
            swerveSubsystem::getPose,
            // Chassis kinematics
            DriveConstants.kDriveKinematics,
            // PID controllers
            xController,
            yController,
            thetaController,
            // Sets swerve module states
            swerveSubsystem::setModuleStates,
            // Required swerve subsystem
            swerveSubsystem);

        new SequentialCommandGroup(
            // Resets odometer before autonomous command starts
            // By resetting the odometer, the trajectory moves to where the robot is
            new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
            swerveControllerCommand,
            // Stops swerve modules after autonomous command finishes
            new InstantCommand(() -> swerveSubsystem.stopModules()));

    }
        

}
