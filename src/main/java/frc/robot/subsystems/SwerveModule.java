// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {
    // **IMPORTANT** This is only for ONE SparkMax swerve module
    // To install these types (like the CAN) go here: https://docs.revrobotics.com/sparkmax/software-resources/spark-max-api-information

    // This is an example with 2 neo motors with a spark max motor controller
    private final CANSparkMax driveMotor;  
    private final CANSparkMax turningMotor;

    // We access these encoders that are built into the motors
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    // PID controller for the motor that controls the angle
    private final PIDController turningPidController;

    /* Absolute encoder connected to the turning motor (in between the 2 big neo motors)
     * This is necessary so the robot always knows where the robot is facing
     * We use the AnalogInput class because the absolute encoders are connected to the analog in ports on the RoboRIO 
     * Make sure to import the one from WPI, not REV*/ 
    private final AnalogInput absoluteEncoder;
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
    absoluteEncoder = new AnalogInput(absoluteEncoderID);

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
    turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
    // Create a PID controller for controlling a system with continuous angles from -π to π radians
    // This setup is suitable for systems with circular behavior, where angles wrap around like a circle 
    turningPidController.enableContinuousInput(-Math.PI, Math.PI);

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
    double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
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

  @Override
  public void periodic() {
    //
  }
}
