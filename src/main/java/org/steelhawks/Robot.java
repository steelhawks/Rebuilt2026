package org.steelhawks;

import au.grapplerobotics.CanBridge;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.FRCNetComm;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.util.ClassPreloader;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.steelhawks.generated.TunerConstants;
import org.steelhawks.generated.TunerConstantsAlpha;
import org.steelhawks.generated.TunerConstantsChassis;
import org.steelhawks.generated.TunerConstantsLastYear;
import org.steelhawks.Constants.Mode;
import org.steelhawks.simulation.FuelPhysicsSim;
import org.steelhawks.simulation.ProjectileSim;
import org.steelhawks.simulation.ShotCalculator;
import org.steelhawks.subsystems.intake.IntakeConstants;
import org.steelhawks.subsystems.superstructure.SuperstructureVisualizer;
import org.steelhawks.subsystems.vision.VisionConstants;
import org.steelhawks.util.Elastic;
import org.steelhawks.util.LoopTimeUtil;
import org.steelhawks.util.PhoenixUtil;
import org.steelhawks.util.VirtualSubsystem;

import java.lang.reflect.Field;

import static org.steelhawks.Constants.RobotType.*;


public class Robot extends LoggedRobot {

    private static final double loopOverrunWarningTimeout = 0.2;
    private static RobotState mState = RobotState.DISABLED;
    private final RobotContainer robotContainer;
    private static boolean isFirstRun = true;
    private Command autonomousCommand;

    public enum RobotState {
        DISABLED,
        TELEOP,
        AUTON,
        TEST
    }

    private void setState(RobotState state) {
        mState = state;
        Logger.recordOutput("RobotState", state.toString());
    }

    public static RobotState getState() {
        return mState;
    }

    public static boolean isFirstRun() {
        return isFirstRun;
    }

    public static ShotCalculator.Config shotCalcConfig = new ShotCalculator.Config();
    public static ShotCalculator shotCalc = new ShotCalculator(shotCalcConfig);
    ProjectileSim.SimParameters params = new ProjectileSim.SimParameters(
        0.215,   // ball mass kg
        0.1501,  // ball diameter m
        0.47,    // drag coeff (smooth sphere)
        0.2,     // Magnus coeff
        1.225,   // air density
        0.43,    // exit height (m), floor to where the ball leaves the shooter
        0.1016,  // flywheel diameter (m), measure with calipers
        1.83,    // target height (m), from game manual
        0.6,     // slip factor (0=no grip, 1=perfect), tune this on the real robot
        45.0,    // launch angle from horizontal, measure from CAD
        0.001,   // sim timestep
        1500, 6000, 25, 5.0  // RPM search range, iterations, max sim time
    );
    ProjectileSim sim = new ProjectileSim(params);
    ProjectileSim.GeneratedLUT lut = sim.generateLUT();
    FuelPhysicsSim ballSim = new FuelPhysicsSim("Sim/Fuel");



    @SuppressWarnings("resource")
    public Robot() {
        VisionConstants.APRIL_TAG_LAYOUT.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide); // apriltag field layout is slow, so invoke it to warmup even though blue is default
        SignalLogger.enableAutoLogging(false); // dont log when on fms
        LiveWindow.disableAllTelemetry();
        for (int i = 5800; i < 5810; i++) {
            PortForwarder.add(i, "10.26.1.11", i);
            PortForwarder.add(i, "10.26.1.12", i);
        }

        // record GIT data
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        switch (BuildConstants.DIRTY) {
            case 0:
                Logger.recordMetadata("GitDirty", "All changes committed");
                break;
            case 1:
                Logger.recordMetadata("GitDirty", "Uncommitted changes");
                break;
            default:
                Logger.recordMetadata("GitDirty", "Unknown");
                break;
        }

        HAL.report(
            FRCNetComm.tResourceType.kResourceType_Language,
            FRCNetComm.tInstances.kLanguage_Kotlin,
            0,
            WPILibVersion.Version);
        HAL.report(
            FRCNetComm.tResourceType.kResourceType_Framework,
            FRCNetComm.tInstances.kFramework_AdvantageKit,
            0,
            WPILibVersion.Version);
        DriverStation.silenceJoystickConnectionWarning(true);
        Logger.recordMetadata("Robot", Constants.ROBOT_NAME);
        Logger.recordMetadata("Robot Mode", Constants.getMode().toString());
        Logger.recordMetadata("Robot Type", Constants.getRobot().toString());

        // Set up data receivers & replay source
        switch (Constants.getMode()) {
            case REAL -> {
                // Running on a real robot, log to a USB stick ("/U/logs")
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
                new PowerDistribution(
                    Constants.POWER_DISTRIBUTION_CAN_ID, Constants.PD_MODULE_TYPE); // Enables power distribution logging
            }
            case SIM -> // Running a physics simulator, log to NT
                Logger.addDataReceiver(new NT4Publisher());
            case REPLAY -> {
                // Replaying a log, set up replay source
                setUseTiming(false); // Run as fast as possible
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
            }
        }

        Logger.start();

        // Adjust loop overrun warning timeout
        try {
            Field watchdogField = IterativeRobotBase.class.getDeclaredField("m_watchdog");
            watchdogField.setAccessible(true);
            Watchdog watchdog = (Watchdog) watchdogField.get(this);
            watchdog.setTimeout(loopOverrunWarningTimeout);
        } catch (Exception e) {
            DriverStation.reportWarning("Failed to disable loop overrun warnings.", false);
        }
        CommandScheduler.getInstance().setPeriod(loopOverrunWarningTimeout);

        ClassPreloader.preload(
            "edu.wpi.first.math.geometry.Transform2d",
            "edu.wpi.first.math.geometry.Twist2d",
            "java.lang.FdLibm$Hypot",
            "choreo.trajectory.Trajectory",
            "choreo.trajectory.SwerveSample");

        // Check for valid swerve config
        var modules =
            switch (Constants.getRobot()) {
                case OMEGABOT, SIMBOT ->
                    new SwerveModuleConstants[]{
                        TunerConstants.FrontLeft,
                        TunerConstants.FrontRight,
                        TunerConstants.BackLeft,
                        TunerConstants.BackRight
                    };
                case ALPHABOT ->
                    new SwerveModuleConstants[]{
                        TunerConstantsAlpha.FrontLeft,
                        TunerConstantsAlpha.FrontRight,
                        TunerConstantsAlpha.BackLeft,
                        TunerConstantsAlpha.BackRight
                    };
                case CHASSIS ->
                    new SwerveModuleConstants[]{
                        TunerConstantsChassis.FrontLeft,
                        TunerConstantsChassis.FrontRight,
                        TunerConstantsChassis.BackLeft,
                        TunerConstantsChassis.BackRight
                    };
                case LAST_YEAR ->
                    new SwerveModuleConstants[]{
                        TunerConstantsLastYear.FrontLeft,
                        TunerConstantsLastYear.FrontRight,
                        TunerConstantsLastYear.BackLeft,
                        TunerConstantsLastYear.BackRight
                    };
                case TEST_BOARD -> null;
            };

        if (RobotConfig.getConfig().hasSwerve) {
            for (var constants : modules) {
                if (constants.DriveMotorType != DriveMotorArrangement.TalonFX_Integrated
                    || constants.SteerMotorType != SteerMotorArrangement.TalonFX_Integrated) {
                    throw new RuntimeException(
                        "You are using an unsupported swerve configuration, which this template does not support without manual customization. The 2025 release of Phoenix supports some swerve configurations which were not available during 2025 beta testing, preventing any development and support from the AdvantageKit developers.");
                }
            }
        }
        robotContainer = new RobotContainer();

        if (Constants.getRobot() == SIMBOT) {
            DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
            DriverStationSim.notifyNewData();
        }
        // ENABLE ONLY IF YOU KNOW WHAT YOU ARE DOING
//        Threads.setCurrentThreadPriority(true, 10);
        CanBridge.runTCP();
    }

    @Override
    public void robotInit() {
        ProjectileSim sim = new ProjectileSim(params);
        ProjectileSim.GeneratedLUT lut = sim.generateLUT();

        for (ProjectileSim.LUTEntry entry : lut.entries()) {
            if (entry.reachable()) {
                shotCalc.loadLUTEntry(entry.distanceM(), entry.rpm(), entry.tof());
            }
        }
    }

    @Override
    public void robotPeriodic() {
        LoopTimeUtil.reset();

        PhoenixUtil.refreshAll();
        LoopTimeUtil.record("PhoenixUtil");
        VirtualSubsystem.periodicAll();
        LoopTimeUtil.record("VirtualPeriodic");
        CommandScheduler.getInstance().run();
        LoopTimeUtil.record("Commands");

        if (Constants.getRobot() == SIMBOT || Toggles.debugMode.get()) {
            visualizeFieldConstants();
        }
        LoopTimeUtil.record("RobotPeriodic");

        if ((Constants.getMode() == Mode.SIM)
            || (!RobotConfig.getConfig().hasSwerve && RobotBase.isReal())
        ) {
            RobotContainer.s_Swerve.updatePhysicsSimulation();
            Translation2d hubCenter = new Translation2d(4.6, 4.0);  // your target
            Translation2d hubForward = new Translation2d(1, 0);       // which way the hub faces

            ShotCalculator.ShotInputs inputs = new ShotCalculator.ShotInputs(
                RobotContainer.s_Swerve.getPose(),
                RobotContainer.s_Swerve.getFieldRelativeSpeeds(),
                RobotContainer.s_Swerve.getChassisSpeeds(),
                hubCenter,
                hubForward,
                0.9,  // vision confidence, 0 to 1
                RobotContainer.s_Swerve.getPitch().getDegrees(),  // pitch for tilt gate (0.0 if no gyro)
                RobotContainer.s_Swerve.getRoll().getDegrees()    // roll for tilt gate (0.0 if no gyro)
            );

            ShotCalculator.LaunchParameters shot = shotCalc.calculate(inputs);
            if (shot.isValid() && shot.confidence() > 50) {
                RobotContainer.s_Flywheel.setTargetVelocity(shot.rpm());
                RobotContainer.s_Turret.setDesiredRotation(shot.driveAngle());
                Translation3d launchPosition = new Translation3d(new Translation2d(
                    SuperstructureVisualizer.turretOffset.getX(),
                    SuperstructureVisualizer.turretOffset.getY())
                );
                double speed =
                    (shot.rpm() / 60) * (SubsystemConstants.OmegaBot.FLYWHEEL.flywheelRadius() * params.slipFactor());
                double launchRad = Units.degreesToRadians(RobotContainer.s_Hood.getPositionDeg());
                double horizontal = speed * Math.cos(launchRad);
                double vertical = speed * Math.sin(launchRad);
                Rotation2d heading = org.steelhawks.RobotState.getInstance().getEstimatedPose().getRotation();
                Translation3d launchVelocity = new Translation3d(
                    horizontal * heading.getCos(),
                    horizontal * heading.getSin(),
                    vertical
                );
                ballSim.launchBall(
                    launchPosition,
                    launchVelocity,
                    shot.rpm());
//                shot.driveAngularVelocityRadPerSec(); gives you a heading feedforward if you want it
            }
        }
    }

    private void visualizeFieldConstants() {
        Constants.toLoggedPoint("StartLine", FieldConstants.Ferrying.START_LINE);
        Constants.toLoggedPoint("EndLine", FieldConstants.Ferrying.END_LINE);
    }

    public void resetPoseSim() {
//        if (Constants.getMode() == Mode.SIM) {
//            RobotContainer.s_Swerve.resetSimulation(
//                ChoreoTraj.OutpostTrenchClimbAuto.initialPoseBlue());
//        }
    }

    @Override
    public void disabledInit() {
        setState(RobotState.DISABLED);
        resetPoseSim();
    }

    @Override
    public void disabledExit() {
        isFirstRun = false;
    }

    @Override
    public void autonomousInit() {
        setState(RobotState.AUTON);
        Elastic.selectTab("Autonomous");
        autonomousCommand = Autos.getAuto();


        if (autonomousCommand != null)
            CommandScheduler.getInstance().schedule(autonomousCommand);
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        setState(RobotState.TELEOP);
        Elastic.selectTab("Teleoperated");
        if (autonomousCommand != null)
            autonomousCommand.cancel();
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        setState(RobotState.TEST);
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void simulationInit() {
        if (Constants.getMode() == Mode.SIM) {
            RobotContainer.s_Swerve.resetSimulation(new Pose2d(0, 0, new Rotation2d()));
        }
        ballSim.enable();
        ballSim.placeFieldBalls();

        ballSim.configureRobot(
            Constants.RobotConstants.ROBOT_WIDTH_WITH_BUMPERS,
            Constants.RobotConstants.ROBOT_LENGTH_WITH_BUMPERS,
            Units.inchesToMeters(6.0),
            () -> RobotContainer.s_Swerve.getPose(), () -> RobotContainer.s_Swerve.getChassisSpeeds());
    }

    @Override
    public void simulationPeriodic() {
        ballSim.tick();
    }
}
