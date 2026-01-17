

# Swerve Base Template :: Steel Hawks
![Logo](https://www.steelhawks.net/logo.svg)


## Authors

- [@steelhawks](https://www.github.com/steelhawks)
- [@farhanj2](https://www.github.com/farhanj2)


## Acknowledgements

 - [WPILIB Version: v2025.1.1](https://github.com/wpilibsuite/allwpilib/releases/tag/v2025.1.1)



## Project Structure

# Project File Structure

```plaintext
SwerveTemplate/
├── src/
│   ├── main/
│   │   │
│   │   deploy/
│   │   │
│   │   └── java/
│   │       ├── org.steelhawks/
│   │       │   ├── commands/
│   │       │   │   ├── SuperStructure.java   # The command factory for commands that require multiple subsystems
│   │       │   │   ├── DriveCommands.java    # The command factory for commands for driving the robot, including pathfinding and auton
│   │       │   ├── generated/                # The folder with the generated TunerConstants from CTRE's Swerve tuner on Phoenix Tuner X
│   │       │   │   ├── TunerConstants.java    # The generated TunerConstants from CTRE's Swerve tuner on Phoenix Tuner X
│   │       │   ├── util/                     # The folder with all custom libraries, such as vision with Limelight, and implementations in code such as odometry.]
│   │       │   │   ├── AllianceFlip.java     # A helper class that flips the robot's pose coordinates and rotation based on the alliance
│   │       │   │   ├── Conversions.java      # A helper class in converting between obscure units
│   │       │   │   ├── DashboardTrigger.java # A helper class that triggers a command when a button is pressed from the iPad app HawkControl. This can be used in tandem with a controller.
│   │       │   │   ├── LocalADStarAK.java    # A helper class that implements the A* algorithm for pathfinding
│   │       │   │   ├── PhoenixUtil.java      # A helper class that contains utility functions for Phoenix motor controllers
│   │       │   │   ├── TunableNumber.java    # A helper class which allows for tuning of numbers in the AdvantageScope, without redeploying each time a number is changed
│   │       │   ├── subsystems/               # The main folder where all the subsystem logic is written
│   │       │   │    ├── swerve/              # The folder with all the swerve logic
│   │       │   │    │    ├── GyroIO.java                 # The IO Interface for the Gyro
│   │       │   │    │    ├── GyroIONavX.java             # The IO Implementation for the NavX
│   │       │   │    │    ├── GyroIOPigeon2.java          # The IO Implementation for the Pigeon2 (Most likely the main gyro)
│   │       │   │    │    ├── ModuleIO.java               # The IO Interface for a SwerveModule
│   │       │   │    │    ├── ModuleIOSim.java            # The IO Implementation for a SwerveModule in the simulator
│   │       │   │    │    ├── ModuleIOTalonFX.java        # The IO Implementation for a SwerveModule for TalonFX motors
│   │       │   │    │    ├── PhoenixOdometryThread.java  # A daemon thread that runs the odometry calculations
│   │       │   │    │    ├── Swerve.java                 # The main subsystem that controls the movement of the drivetrain
│   │       │   │    │    ├── SwerveModule.java           # The helper file for a SwerveModule, taking in a ModuleIO in its constructor
│   │       │   │    ├── LED.java             # The subsystem that controls the LEDs
│   │       │   ├── Constants.java            # Configuration for motor ports, speeds, etc
│   │       │   ├── Autos.java                # Configuration for Autonomous commands and to use vision or not
│   │       │   ├── BuildConstants.java       # Gets the Git status of the code such as if it is dirty, time of commit, etc. This is generated when code is deployed
│   │       │   ├── SwerveModule.java         # Controls individual module's speed and direction
│   │       │   ├── OperatorLock.java         # Enum to control different key mappings
│   │       │   ├── RobotMode.java            # Enum to control different key mappings
│   │       │   └── RobotContainer.java       # Manages controls and subsystems
│   └── test/
│       └── java/                                 # Unit tests (if any)
├── vendordeps/                                   # External vendor libraries
├── build.gradle                                  # Gradle build file
├── settings.gradle                               # Gradle settings
└── WPILib-License.md                             # WPILib license info
```

## Explanation of Project Structure

### Root Directory
- **`build.gradle`**: Defines project dependencies, build tasks, and configuration for Gradle.
- **`settings.gradle`**: Specifies project settings and root directory info for Gradle.
- **`vendordeps/`**: Contains vendor dependency files, crucial for hardware interfaces (e.g., CTRE or Rev Robotics libraries).
- **`WPILib-License.md`**: Licensing information for WPILib, which provides the core libraries for FRC robot programming.

### `src/main/java/org/steelhawks/`
- **`Constants.java`**: Holds all constant values, such as motor ports, PID settings, and other configurations.
- **`SwerveModule.java`**: Manages each swerve module’s direction and speed, allowing independent control over each wheel.
- **`RobotContainer.java`**: Links subsystems, commands, and input devices, acting as the central command hub for the robot code.
- **`OperatorLock.java`** & **`RobotMode.java`**: Enumerations that handle different control modes and operator mappings for flexible user input setups.

### Commands
- **`commands/SuperStructure.java`**: A command factory for handling complex commands involving multiple subsystems, making it easier to coordinate between drivetrain, intake, and shooter, if applicable.
- **`commands/DriveCommands.java`**: A command factory for handling driving commands such as pathfinding, autonomous routines, and other drivetrain-specific tasks.

### `subsystems/`
- **`Swerve.java`**: Main subsystem that manages the swerve drive logic, allowing the robot to move omnidirectionally by coordinating individual `SwerveModule` instances.

### `util/`
- **Utility Code**: This folder is designed to store helper classes and utilities that support core functionality, like math utilities, sensor handling, or other common tasks.

### `src/test/java`
- **Testing**: Contains unit tests or simulation-based tests, allowing verification of subsystem and command behavior in a safe environment.

# Tuning
<p>When you make a clone of this repo, make sure to change the following.</p>
<p>1. Set the name of your robot at ROBOT_NAME in Constants.java</p>
<p>2. Identify and tune the swerve drive using CTRE's Swerve Drive Generator. Information on this can be found <a href="https://v6.docs.ctr-electronics.com/en/2024/docs/tuner/tuner-swerve/creating-your-project.html">here</a>.</p>
<p>3. Once done, start tuning Pathplanner. This can be done by first creating a straight line to tune the translation PID, then using that same line, but adding a full 360º turn to tune the rotation PID.</p>
<p>4. Do a full and through recheck of all your code to see if anything is off or incorrect before proceeding to writing more code. This will save you in the long run.</p>
