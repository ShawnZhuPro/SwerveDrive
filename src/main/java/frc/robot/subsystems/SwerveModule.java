// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
    // **IMPORTANT** This is only for ONE SparkMax swerve module
    // To install these types (like the CAN) go here: https://docs.revrobotics.com/sparkmax/software-resources/spark-max-api-information

    // This is an example with 2 neo motors with a spark max motor controller
    private final CANSparkMax driveMotor;  
    private final CANSparkMax turningMotor;

    // We access these encoders that are built into the motors
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;  // Switch to CANCoder

    // PID controller for the motor that controls the angle
    private final PIDController turningPIDController;

    /* Absolute encoder connected to the turning motor (in between the 2 big neo motors)
     * This is necessary so the robot always knows where the robot is facing
     * In other teams' code, this may be named "angleEncoder" */ 
    private final CANCoder absoluteEncoder;
    // Stores if the absoluteEncoder is reversed
    private final boolean absoluteEncoderReversed;
    /* Offset position of absoluteEncoder
     * This is important because when we first assemble the chassis, the absolute encoder reading will differ from the actual wheel angle
     * By storing this offset in a variable, we can compensate for this later in the code */
    private final double absoluteEncoderOffsetRad;

  // SwerveModule class constructor
  public SwerveModule(int driveMotorID, int turningMotorID, boolean driveMotorReversed, boolean turningMotorReversed,
                    int absoluteEncoderID, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
    
    // Create absolute encoder
    this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
    this.absoluteEncoderReversed = absoluteEncoderReversed;
    absoluteEncoder = new CANCoder(absoluteEncoderID);

    // Create motors
    driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
    turningMotor = new CANSparkMax(turningMotorID, MotorType.kBrushless);

    driveMotor.setInverted(driveMotorReversed);
    turningMotor.setInverted(turningMotorReversed);

    // Get motor encoders
    driveEncoder = driveMotor.getEncoder();
    turningEncoder = turningMotor.getEncoder();

    /* Set encoder conversion constants to work with meters & radians rather than rotations
     * 'k' means that it's a constant
     * Go to Constants.java to see the numbers */
    driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
    driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
    turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
    turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

    // Initialize turningPidController to control the system's angle
    // As of now, we'll see if the proportional term alone works
    turningPIDController = new PIDController(ModuleConstants.kPTurning, 0, 0);
    // Create a PID controller for controlling a system with continuous angles from -π to π radians
    // This setup is suitable for systems with circular behavior, where angles wrap around like a circle 
    turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

    // We call this so the turningEncoder's reading will be aligned with the wheel's actual angle (refer to resetEncoders method down below)
    resetEncoders();

  }

  // Helper methods to get encoder values
  public double getDrivePosition(){
    return driveEncoder.getPosition();
  }

  public double getTurningPosition(){
    return turningEncoder.getPosition();
  }

  public double getDriveVelocity(){
    return driveEncoder.getVelocity();
  }

  public double getTurningVelocity(){
    return turningEncoder.getVelocity();
  }

  public double getAbsoluteEncoderRad(){
    // absoluteEncoder's voltage reading / voltage supply 
    // This gives us the % of a full rotation
    double angle = absoluteEncoder.getBusVoltage() / RobotController.getVoltage5V();
    // Convert to radians
    angle *= 2.0 * Math.PI;
    // Actual angle by subtracting the offset
    angle -= absoluteEncoderOffsetRad;
    // if the absoluteEncoder is reversed, we multiply the angle by -1
    if(absoluteEncoderReversed){
        return angle * -1.0;
    }return angle * 1.0;
  }

  // Encoders in the motors lose their readings everytime the robot starts
  // This function gives values of the absoluteEncoders (which always knows their location) to the motor encoders
  public void resetEncoders(){
    // Resets driveEncoder motor to 0 
    driveEncoder.setPosition(0);
    // Resets turningEncoder to the absoluteEncoder in radians
    turningEncoder.setPosition(getAbsoluteEncoderRad());
  }

  // WPI libraries need this information in a SwerveModuleState, so let's create a method to do so
  public SwerveModuleState getState(){
      // Create a new SwerveModuleState with the drive velocity and turning position
      return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
  }

  // Function that actuates (controls and adjusts the orientation) the swerve module
  public void setDesiredState(SwerveModuleState state){
    /* When using joysticks to drive the robot, the WPI library resets the module angles to zero as soon as the controls are released, causing unexpected behavior
     * To fix this, we will stop the motors to prevent unnecessary movement if the swerve module has no substantial driving velocity in the new requested state
     * If the velocity is very close to zero, it means we are not actively driving the module, so we can reset the motors */ 
    if (Math.abs(state.speedMetersPerSecond) < 0.001) {
        stop();
        return;  // Exits the setDesiredState function
    }

    // Optimizes the angle setpoint so we don't have to move more than 90 degrees
    state = SwerveModuleState.optimize(state, getState().angle);

    // Sets the driveMotor
    // Scales down the velocity using the robot's max speed into a power output rather than a speed
    driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

    // Sets the turningMotor
    // Calculates the output for the current position (1st argument) and the angle setpoint (2nd argument)
    turningMotor.set(turningPIDController.calculate(getTurningPosition(), state.angle.getRadians()));

    // Debug info about the state of the specific swerve module
    SmartDashboard.putString("Swerve[" + absoluteEncoder.getDeviceID() + "] state", state.toString());
  }

  // Sets drive and angle speeds to 0 (stops the module)
  public void stop(){
    driveMotor.set(0);
    turningMotor.set(0);
  }
}
