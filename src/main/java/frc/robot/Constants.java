// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  // Change these constants to ones measured for your robot
  public static final class ModuleConstants {
    // Diameter of the swerve module's wheels in meters
    public static final double kWheelDiameterMeters = 0;

    // Gear ratios for drive and turning motors
    public static final double kDriveMotorGearRatio = 0;
    public static final double kTurningMotorGearRatio = 0;

    // Calculate the conversion factor from drive motor encoder rotations to meters
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;

    // Calculate the conversion factor from turning motor encoder rotations to radians
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;

    // Calculate the conversion factor from drive motor encoder RPM to meters per second
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;

    // Calculate the conversion factor from turning motor encoder RPM to radians per second
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;

    // Proportional tuning constant for turning control
    public static final double kPTurning = 0;
  }

  // Change these constants to ones measured for your robot
  public static final class DriveConstants {
    // Port numbers for drive motors on different swerve modules
    public static final int kFrontLeftDriveMotorPort = 0;
    public static final int kBackLeftDriveMotorPort = 0;
    public static final int kFrontRightDriveMotorPort = 0;
    public static final int kBackRightDriveMotorPort = 0;

    // Port numbers for turning motors on different swerve modules
    public static final int kFrontLeftTurningMotorPort = 0;
    public static final int kBackLeftTurningMotorPort = 0;
    public static final int kFrontRightTurningMotorPort = 0;
    public static final int kBackRightTurningMotorPort = 0;

    // Flags to reverse the turning encoder direction on swerve modules
    public static final boolean kFrontLeftTurningEncoderReversed = true;
    public static final boolean kBackLeftTurningEncoderReversed = true;
    public static final boolean kFrontRightTurningEncoderReversed = true;
    public static final boolean kBackRightTurningEncoderReversed = true;

    // Flags to reverse the drive encoder direction on swerve modules
    public static final boolean kFrontLeftDriveEncoderReversed = true;
    public static final boolean kBackLeftDriveEncoderReversed = true;
    public static final boolean kFrontRightDriveEncoderReversed = true;
    public static final boolean kBackRightDriveEncoderReversed = true;

    // Port numbers for absolute drive encoders on swerve modules
    public static final int kFrontLeftDriveAbsoluteEncoderPort = 0;
    public static final int kBackLeftDriveAbsoluteEncoderPort = 0;
    public static final int kFrontRightDriveAbsoluteEncoderPort = 0;
    public static final int kBackRightDriveAbsoluteEncoderPort = 0;

    // Flags to reverse the absolute drive encoder direction on swerve modules
    public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = true;
    public static final boolean kBackLeftDriveAbsoluteEncoderReversed = true;
    public static final boolean kFrontRightDriveAbsoluteEncoderReversed = true;
    public static final boolean kBackRightDriveAbsoluteEncoderReversed = true;

    // Offset angles in radians for absolute drive encoders on swerve modules
    public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0;
    public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0;
    public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 0;
    public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 0;

    // Maximum physical speed in meters per second
    public static final double kPhysicalMaxSpeedMetersPerSecond = 0;
  }

}
