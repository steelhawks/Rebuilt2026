package org.steelhawks.hawklights;

/**
 * Period lengths used by the match-clock timer patterns, in seconds. Defaults
 * are standard FRC ({@code auton=15}, {@code teleop=135}). Override them when
 * your event uses non-standard timings or for off-season demos.
 *
 * <pre>
 *   // standard
 *   LedPattern.matchTimer();
 *
 *   // custom: 20s auton, 120s teleop
 *   LedPattern.matchTimer(new MatchTimings(20, 120));
 * </pre>
 */
public class MatchTimings {

  /** Standard FRC autonomous length, seconds. */
  public static final double DEFAULT_AUTON_SEC = 15.0;

  public static final double REBUILT_AUTON_SEC = 20.0;

  /** Standard FRC teleop length, seconds. */
  public static final double DEFAULT_TELEOP_SEC = 135.0;

  public final double autonSec;
  public final double teleopSec;

  public MatchTimings(double autonSec, double teleopSec) {
    this.autonSec = autonSec;
    this.teleopSec = teleopSec;
  }

  /** Standard FRC timings: 15s auton, 135s teleop (2:15). */
  public static MatchTimings standard() {
    return new MatchTimings(DEFAULT_AUTON_SEC, DEFAULT_TELEOP_SEC);
  }

  /** Total match length (auton + teleop), seconds. */
  public double totalSec() {
    return autonSec + teleopSec;
  }
}
