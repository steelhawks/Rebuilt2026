package org.steelhawks;

import org.steelhawks.subsystems.indexer.Indexer;
import org.steelhawks.subsystems.intake.Intake;
import org.steelhawks.subsystems.led.LEDMatrix;
import org.steelhawks.subsystems.led.LEDStrip;
import org.steelhawks.subsystems.oldintake.OldIntake;
import org.steelhawks.subsystems.superstructure.flywheel.Flywheel;
import org.steelhawks.subsystems.superstructure.hood.Hood;
import org.steelhawks.subsystems.superstructure.turret.Turret;
import org.steelhawks.subsystems.swerve.Swerve;
import org.steelhawks.subsystems.vision.Vision;
import org.steelhawks.subsystems.vision.objdetect.ObjectVision;

import java.util.Optional;

/**
 * Typed registry of subsystems for the current robot.
 *
 * <p>Lifecycle:
 * <ol>
 *   <li>{@code init()} creates an empty registry. Called by RobotContainer
 *       <em>before</em> any subsystem is constructed, so cross-references that
 *       fire during construction (e.g. {@link org.steelhawks.RobotState}'s
 *       footprint extension supplier) see Optional.empty() instead of a
 *       not-installed exception.</li>
 *   <li>RobotContainer assigns each subsystem via {@code setX(...)} as it
 *       constructs it.</li>
 *   <li>By the time bindings are configured, all subsystems are populated.</li>
 * </ol>
 *
 * <p>Two access patterns:
 * <ul>
 *   <li>{@code Subsystems.hood()} — direct accessor, throws with a descriptive
 *       error if the subsystem isn't configured for this robot. The default;
 *       use everywhere the subsystem is logically required.</li>
 *   <li>{@code Subsystems.hoodIfPresent()} — returns {@code Optional<Hood>}.
 *       Use only where missing-subsystem is a legitimate runtime semantic
 *       (binding setup, dashboard overrides, footprint extensions).</li>
 * </ul>
 */
public final class Subsystems {

    private static Subsystems instance;

    /** Create and install an empty registry. Call once, before subsystem construction. */
    public static Subsystems init() {
        instance = new Subsystems();
        return instance;
    }

    /** Test/teardown hook. */
    public static void install(Subsystems s) {
        instance = s;
    }

    public static Subsystems get() {
        if (instance == null) {
            throw new IllegalStateException(
                "Subsystems registry not yet installed — RobotContainer hasn't started constructing");
        }
        return instance;
    }

    private Swerve swerve;
    private Vision vision;
    private ObjectVision objVision;
    private Flywheel flywheel;
    private Turret turret;
    private Hood hood;
    private Intake intake;
    private OldIntake oldIntake;
    private Indexer indexer;
    private LEDMatrix matrix;
    private LEDStrip strip;

    private Subsystems() {}

    public Subsystems setSwerve(Swerve s)        { this.swerve = s; return this; }
    public Subsystems setVision(Vision v)        { this.vision = v; return this; }
    public Subsystems setObjVision(ObjectVision o){ this.objVision = o; return this; }
    public Subsystems setFlywheel(Flywheel f)    { this.flywheel = f; return this; }
    public Subsystems setTurret(Turret t)        { this.turret = t; return this; }
    public Subsystems setHood(Hood h)            { this.hood = h; return this; }
    public Subsystems setIntake(Intake i)        { this.intake = i; return this; }
    public Subsystems setOldIntake(OldIntake o)  { this.oldIntake = o; return this; }
    public Subsystems setIndexer(Indexer i)      { this.indexer = i; return this; }
    public Subsystems setMatrix(LEDMatrix m)     { this.matrix = m; return this; }
    public Subsystems setStrip(LEDStrip s)       { this.strip = s; return this; }

    // ---- Throwing accessors (the default) ----

    public static Swerve swerve()             { return require(get().swerve, "Swerve"); }
    public static Vision vision()             { return require(get().vision, "Vision"); }
    public static ObjectVision objVision()    { return require(get().objVision, "ObjectVision"); }
    public static Flywheel flywheel()         { return require(get().flywheel, "Flywheel"); }
    public static Turret turret()             { return require(get().turret, "Turret"); }
    public static Hood hood()                 { return require(get().hood, "Hood"); }
    public static Intake intake()             { return require(get().intake, "Intake"); }
    public static OldIntake oldIntake()       { return require(get().oldIntake, "OldIntake"); }
    public static Indexer indexer()           { return require(get().indexer, "Indexer"); }
    public static LEDMatrix matrix()          { return require(get().matrix, "LEDMatrix"); }
    public static LEDStrip strip()            { return require(get().strip, "LEDStrip"); }

    // ---- Optional accessors: only for sites where absence is a legitimate skip ----

    public static Optional<Vision> visionIfPresent()         { return Optional.ofNullable(get().vision); }
    public static Optional<ObjectVision> objVisionIfPresent(){ return Optional.ofNullable(get().objVision); }
    public static Optional<Flywheel> flywheelIfPresent()     { return Optional.ofNullable(get().flywheel); }
    public static Optional<Turret> turretIfPresent()         { return Optional.ofNullable(get().turret); }
    public static Optional<Hood> hoodIfPresent()             { return Optional.ofNullable(get().hood); }
    public static Optional<Intake> intakeIfPresent()         { return Optional.ofNullable(get().intake); }
    public static Optional<OldIntake> oldIntakeIfPresent()   { return Optional.ofNullable(get().oldIntake); }
    public static Optional<Indexer> indexerIfPresent()       { return Optional.ofNullable(get().indexer); }
    public static Optional<LEDMatrix> matrixIfPresent()      { return Optional.ofNullable(get().matrix); }
    public static Optional<LEDStrip> stripIfPresent()        { return Optional.ofNullable(get().strip); }

    private static <T> T require(T s, String name) {
        if (s == null) {
            throw new IllegalStateException(
                name + " is not configured for robot " + Constants.getRobot()
                    + ". If this access is legitimate for this robot, switch to "
                    + Character.toLowerCase(name.charAt(0)) + name.substring(1) + "IfPresent().");
        }
        return s;
    }
}
