### 1) RobotContainer Joystick method deprecation bug
Wrong code:
```java
    private void configureButtonBindings() {
        new JoystickButton(driverJoytick, 2).whenPressed(() -> swerveSubsystem.zeroHeading());
    }
```
Fixed code:
```java
    // onTrue takes in a command as an argument, so we make a new ZeroHeading command to zero the heading
    // the "() ->" expression is a shorthand way of making commands (AKA lambda expression)
    InstantCommand zeroHeading = new InstantCommand(() -> swerveSubsystem.getHeading());
    // Resets the heading on a specific button click (specified in constants)
    new JoystickButton(driverJoystick, OIConstants.kDriverZeroHeading).onTrue(zeroHeading);
```
Explanation:
```txt
The whenPressed() method was deprecated, so I replaced it with the onTrue() method.
It takes in a command parameter, so I had to create a new command that would zero the heading
using the lambda function and assigning it to an InstantCommand variable.
```

<br>

### 2) SwerveSubsystem initializing and updating odometer values bug
Wrong code:
```java
private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, new Rotation2d(0));
...
public void periodic() {
        odometer.update(getRotation2d(), frontLeft.getState(), frontRight.getState(), backLeft.getState(),
                backRight.getState());
        ...
}
```
Fixed code:
```java
// Initially creates a robotPosition with default values; we will update the position in the periodic function
private SwerveModulePosition[] robotPosition;
// Creates an odometer that measures the coordinates of the robot on the field
private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, new Rotation2d(0), robotPosition);
...
public void periodic() {
    // Creates an array of SwerveModulePosition objects to represent module positions
    SwerveModulePosition[] modulePositions = new SwerveModulePosition[]{
      new SwerveModulePosition(frontLeft.getDrivePosition(), new Rotation2d(frontLeft.getTurningPosition())),
      new SwerveModulePosition(frontRight.getDrivePosition(), new Rotation2d(frontRight.getTurningPosition())),
      new SwerveModulePosition(backLeft.getDrivePosition(), new Rotation2d(backLeft.getTurningPosition())),
      new SwerveModulePosition(backRight.getDrivePosition(), new Rotation2d(backRight.getTurningPosition()))
  };
    // Updates the odometer reading
    odometer.update(getRotation2d(), modulePositions);
    ...
}
```
Explanation:
```txt
The SwerveDriveOdometry class takes in 3 parameters, not 2; the last parameter is the robot's position.
The update() method takes in 2 parameters, not 5; the last parameter is an array of the robot's position.
Therefore, in order to get the robot's position, we have to create an array of the positions of each of the 4 swerve modules.
Now we can successfully apply the arguments to the update() method so that the odometry coordinates are continuously updating.
```

<br>

### 3) Odometer reset position bug
Wrong code:
```java
    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(pose, getRotation2d());
    }
```
Fixed code:
```java
  // Resets the odometer to a new location
  public void resetOdometry(Pose2d pose){
    odometer.resetPosition(getRotation2d(), robotPosition, pose);
  }
```
Explanation:
```txt
The resetPosition() method for the odometer takes in 3 parameters, not 2.
The first parameter is the heading of the robot in Rotation2D (a data structure).
The second parameter is the robot's position according to an array of 4 swerve modules.
The third parameter is the location of the robot determined by the odometer.
```
