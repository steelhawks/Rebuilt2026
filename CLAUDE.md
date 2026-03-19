# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an FRC (First Robotics Competition) swerve drive robot codebase for Team Steel Hawks. It uses WPILib 2025, AdvantageKit for logging, and CTRE Phoenix 6 for motor control.

## Common Commands

```bash
# Build the robot code
./gradlew build

# Deploy to RoboRIO
./gradlew deploy

# Run the simulator
./gradlew simulate

# Run a specific test
./gradlew test --tests "TestClassName.testMethodName"

# Run code formatting (Spotless)
./gradlew spotlessApply

# Build with Java compile only (faster iteration)
./gradlew compileJava
```

## Architecture

### Robot Types and Modes

The codebase supports multiple robot configurations:
- **OMEGABOT** - Primary competition robot
- **ALPHABOT** - Secondary robot
- **SIMBOT** - Simulation robot for physics simulation
- **LAST_YEAR** - Previous year's robot

Modes: `REAL`, `SIM`, `REPLAY`

### IO Pattern (AdvantageKit)

This project uses the AdvantageKit IO pattern for hardware abstraction:
- **IO Interfaces** (e.g., `ModuleIO.java`, `FlywheelIO.java`) - Define the contract for hardware operations
- **IO Implementations** (e.g., `ModuleIOTalonFX.java`, `ModuleIOSim.java`) - Concrete implementations for real hardware or simulation
- This allows easy swapping between real/sim hardware and enables replay from log files

### Key Subsystems

- **Swerve** - Swerve drive control with individual module control (located in `subsystems/swerve/`)
- **Intake** - Game piece intake mechanism (subsystems/intake/)
- **Flywheel** - Shooter flywheel for scoring (subsystems/Superstructure/flywheel/)
- **Vision** - AprilTag vision using PhotonVision/Limelight (subsystems/vision/)
- **LED** - LED strips and matrix for operator feedback (subsystems/led/)

### Command Structure

- **SuperStructure.java** - Command factory for multi-subsystem commands
- **DriveCommands.java** - Driving commands including pathfinding with PathPlanner/Choreo
- **Autos.java** - Autonomous routine configuration

### Generated Constants

The `generated/` folder contains CTRE Phoenix Tuner output (TunerConstants.java). Regenerate via Phoenix Tuner X when modifying hardware.

### Configuration

- **Constants.java** - Robot-wide constants (ports, PID values, etc.)
- **RobotConfig.java** - Factory for creating subsystem instances based on robot type
- **Toggles.java** - Runtime feature flags

### Logging

Uses AdvantageKit (Logger) for structured logging. Data can be analyzed with AdvantageScope. Logs are written to USB stick on real robot, NetworkTables in simulation.