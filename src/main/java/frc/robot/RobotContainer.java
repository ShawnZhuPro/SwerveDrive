// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveAutoTrajectory;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Creates a SwerveSubsystem instance
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  // Creates a joystick object
  private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);
  
  public RobotContainer(){
    // Sets swerve drive as default cmd
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
                () -> driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
                () -> driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
                () -> !driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));  // By default, cmd operates by field's reference frame

    configureBindings();
  }

  private void configureBindings() {
    // onTrue takes in a command as an argument, so we make a new ZeroHeading command to zero the heading
    // the "() ->" expression is a shorthand way of making commands (AKA lambda expression)
    InstantCommand zeroHeading = new InstantCommand(() -> swerveSubsystem.getHeading());
    // Resets the heading on a specific button click (specified in constants)
    new JoystickButton(driverJoystick, OIConstants.kDriverZeroHeading).onTrue(zeroHeading);
  }

  public Command getAutonomousCommand() {
    // Returns the autonomous swerve trajectory command
    return new SwerveAutoTrajectory(swerveSubsystem);
  }
}
