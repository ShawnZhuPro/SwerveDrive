// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {

  // Creates a swerve module object for the front left module
  private final SwerveModule frontLeft = new SwerveModule(
    DriveConstants.kFrontLeftDriveMotorPort,
    DriveConstants.kFrontLeftTurningMotorPort,
    DriveConstants.kFrontLeftDriveEncoderReversed,
    DriveConstants.kFrontLeftTurningEncoderReversed,
    DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
    DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
    DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);
  
  // Creates a swerve module object for the front right module
  private final SwerveModule frontRight = new SwerveModule(
    DriveConstants.kFrontRightDriveMotorPort,
    DriveConstants.kFrontRightTurningMotorPort,
    DriveConstants.kFrontRightDriveEncoderReversed,
    DriveConstants.kFrontRightTurningEncoderReversed,
    DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
    DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
    DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

  // Creates a swerve module object for the back left module
  private final SwerveModule backLeft = new SwerveModule(
    DriveConstants.kBackLeftDriveMotorPort,
    DriveConstants.kBackLeftTurningMotorPort,
    DriveConstants.kBackLeftDriveEncoderReversed,
    DriveConstants.kBackLeftTurningEncoderReversed,
    DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
    DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
    DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

  // Creates a swerve module object for the back right module
  private final SwerveModule backRight = new SwerveModule(
    DriveConstants.kBackRightDriveMotorPort,
    DriveConstants.kBackRightTurningMotorPort,
    DriveConstants.kBackRightDriveEncoderReversed,
    DriveConstants.kBackRightTurningEncoderReversed,
    DriveConstants.kBackRightDriveAbsoluteEncoderPort,
    DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
    DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

  // Creates a NAVX gyroscope
  // You must do import edu.wpi.first.wpilibj.SPI
  private AHRS gyro = new AHRS(SPI.Port.kMXP);

  // Initially creates a robotPosition with default values; we will update the position in the periodic function
  private SwerveModulePosition[] robotPosition;
  // Creates an odometer that measures the coordinates of the robot on the field
  private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, new Rotation2d(0), robotPosition);

  public SwerveSubsystem() {
    // Resets the gyroscope every time the robot boots up
    // This makes it so the direction the robot is facing currently will be the forward direction of the field
    zeroHeading();
//     new Thread(() -> {
//       try {
//           Thread.sleep(1000);
//           zeroHeading();
//       } catch (Exception e) {
//       }
//   }).start();
// }  In case the gyroscope is busy recalibrating when the robot starts, this is a fix that delays zeroHeading by 1 second
  }

  public void zeroHeading() {
    gyro.reset();
  }

  // Default gyroscope values are continuos (meaning it can go to 360 degrees, 720, etc.)
  public double getHeading() {
    // This function restricts the values from -180 to 180 degrees to make it easier to use
    return Math.IEEEremainder(gyro.getAngle(), 360);
  }

  // WPIlib wants the heading (angle) value in rotation 2D (a data structure that represents a 2D rotation angle)
  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  // Get's the location determined by the odometer
  // Has the x, y, and theta coordinates of the robot
  public Pose2d getPose() {
    return odometer.getPoseMeters();
  }

  // Resets the odometer to a new location
  public void resetOdometry(Pose2d pose){
    odometer.resetPosition(getRotation2d(), robotPosition, pose);
  }


  @Override 
  public void periodic() {
    // Creates an array of SwerveModulePosition objects to represent module positions
    SwerveModulePosition[] robotPosition = new SwerveModulePosition[]{
      new SwerveModulePosition(frontLeft.getDrivePosition(), new Rotation2d(frontLeft.getTurningPosition())),
      new SwerveModulePosition(frontRight.getDrivePosition(), new Rotation2d(frontRight.getTurningPosition())),
      new SwerveModulePosition(backLeft.getDrivePosition(), new Rotation2d(backLeft.getTurningPosition())),
      new SwerveModulePosition(backRight.getDrivePosition(), new Rotation2d(backRight.getTurningPosition()))
  };
    // Updates the odometer reading
    odometer.update(getRotation2d(), robotPosition);
  
    // Displays robot heading (angle) value on SmartDashboard or ShuffleBoard
    SmartDashboard.putNumber("Robot Heading", getHeading());
    // Displays robot's coordinates
    SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
  }

  // Stops all swerve modules
  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  // Takes in an array of 4 module states and applies them all
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    /* We have to desaturate (normalize) the wheel speeds to ensure they are within their maximum achievable limits
     * If each wheel has a set speed of 5 meters per second, attempting to drive them over their limits can lead to uncontrollable steering
     * To address this, we use the desaturateWheelSpeeds function to proportionately decrease the wheel speeds, so they are all achievable and under control */
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

    // Applies desired states to individual swerve modules
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }
}
