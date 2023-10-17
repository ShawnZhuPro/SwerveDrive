// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickCmd extends CommandBase {
  
  private final SwerveSubsystem swerveSubsystem;
  
  // A Java supplier is used to provide values/objects without taking any inputs
  // These supplier have a get() method that returns a value
  private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;  // X, Y, and turning speed values
  private final Supplier<Boolean> fieldOrientedFunction;  // Determines if the user wants the joystick cmd to be field-oriented
  
  private SlewRateLimiter xLimiter, yLimiter, turningLimiter;  // These limit the acceleration if the joystick is moved too violently so the robot drives smoother


  // Constructor that "constructs" a SwerveJoystickCmd command given values in the parameters ()
  public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,
  Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
  Supplier<Boolean> fieldOrientedFunction) {
    this.swerveSubsystem = swerveSubsystem;
    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.turningSpdFunction = turningSpdFunction;
    this.fieldOrientedFunction = fieldOrientedFunction;  
    this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // These are continuously updating values (joystick inputs) from the initializes suppliers in the SwerveJoystickCmd constructor
    double xSpeed = xSpdFunction.get();
    double ySpeed = ySpdFunction.get();
    double turningSpeed = turningSpdFunction.get();

    // Applies deadband
    // This ignores small inputs if the joystick doesn't center to zero in order to protect the motors
    if (Math.abs(xSpeed) > OIConstants.kDeadband) {
      xSpeed = xSpeed;
    } else {
      xSpeed = 0.0;
    }
    
    if (Math.abs(ySpeed) > OIConstants.kDeadband) {
      ySpeed = ySpeed;
    } else {
      ySpeed = 0.0;
    }
    
    if (Math.abs(turningSpeed) > OIConstants.kDeadband) {
      turningSpeed = turningSpeed;
    } else {
      turningSpeed = 0.0;
    }

    // These limiters make driving smoother
    // These are scaled down to maxspeed/4 because max speed is too fast
    xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    turningSpeed = turningLimiter.calculate(turningSpeed) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
  

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
