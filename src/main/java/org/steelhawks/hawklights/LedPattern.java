package org.steelhawks.hawklights;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * An immutable LED request: which pattern, what color, and (for time-based
 * patterns) timing info. Build them with the static factories.
 *
 * <pre>
 *   LedPattern.solid(Color.kGreen)
 *   LedPattern.flash(new Color8Bit(255, 0, 0), 200)
 *   LedPattern.rainbow()
 *   LedPattern.matchTimer()                 // tracks the real match clock
 *   LedPattern.countdown(30)                // 30s countdown from when shown
 * </pre>
 *
 * <p>For {@link LedState#TIMER}, {@link #timerTotal()} and {@link
 * #timerRemaining()} are suppliers re-evaluated by {@link HawkLights} every
 * loop, so the bar tracks a live clock and can even change span between
 * autonomous and teleop. Both are {@code null} for non-timer patterns.
 */
public record LedPattern(
    LedState state, Color8Bit color, int intervalMs,
    DoubleSupplier timerTotal, DoubleSupplier timerRemaining) {

  private static final Color8Bit OFF = new Color8Bit(0, 0, 0);
  private static final int DEFAULT_INTERVAL_MS = 250;

  /** Compact constructor for non-timer patterns. */
  public LedPattern(LedState state, Color8Bit color, int intervalMs) {
    this(state, color, intervalMs, null, null);
  }

  public static LedPattern solid(Color8Bit color) {
    return new LedPattern(LedState.SOLID, color, 0);
  }

  public static LedPattern solid(Color color) {
    return solid(new Color8Bit(color));
  }

  public static LedPattern flash(Color8Bit color, int intervalMs) {
    return new LedPattern(LedState.FLASH, color, intervalMs);
  }

  public static LedPattern flash(Color8Bit color) {
    return flash(color, DEFAULT_INTERVAL_MS);
  }

  public static LedPattern flash(Color color, int intervalMs) {
    return flash(new Color8Bit(color), intervalMs);
  }

  public static LedPattern flash(Color color) {
    return flash(new Color8Bit(color));
  }

  public static LedPattern flash(Supplier<Color> color) {
      return flash(color.get());
  }

  public static LedPattern bounce(Color8Bit color) {
    return new LedPattern(LedState.BOUNCE, color, 0);
  }

  public static LedPattern bounce(Color color) {
    return bounce(new Color8Bit(color));
  }

  public static LedPattern fade(Color8Bit color) {
    return new LedPattern(LedState.FADE, color, 0);
  }

  public static LedPattern fade(Color color) {
    return fade(new Color8Bit(color));
  }

  public static LedPattern indicateLeft(Color8Bit color) {
    return new LedPattern(LedState.INDICATE_LEFT, color, 0);
  }

  public static LedPattern indicateRight(Color8Bit color) {
    return new LedPattern(LedState.INDICATE_RIGHT, color, 0);
  }

  /** Full-strip rainbow; color is ignored by the firmware. */
  public static LedPattern rainbow() {
    return new LedPattern(LedState.RAINBOW, OFF, 0);
  }

  // ---- Timer factories ------------------------------------------------------

  /**
   * Countdown bar driven by custom suppliers -- the most flexible form, which
   * everything below is built on. {@code total} is the full bar length and
   * {@code remaining} the live seconds-remaining; both are re-read every loop.
   */
  public static LedPattern timer(DoubleSupplier total, DoubleSupplier remaining) {
    return new LedPattern(LedState.TIMER, OFF, 0, total, remaining);
  }

  /** Fixed-length timer with a live remaining supplier. */
  public static LedPattern timer(double totalSec, DoubleSupplier remaining) {
    return timer(() -> totalSec, remaining);
  }

  /**
   * Fixed-length countdown that starts the moment this pattern first goes active
   * (via {@link HawkLights#show}) and counts {@code totalSec} down to zero.
   */
  public static LedPattern countdown(double totalSec) {
    // Capture the start lazily on first poll so one instance can be reused.
    final double[] start = {-1};
    return timer(
        totalSec,
        () -> {
          double now = Timer.getFPGATimestamp();
          if (start[0] < 0) {
            start[0] = now;
          }
          return Math.max(0, totalSec - (now - start[0]));
        });
  }

  /**
   * Remaining-seconds supplier that uses the real match clock when it's
   * available (FMS or a timed practice match) and otherwise falls back to a
   * wall-clock countdown of {@code total} seconds from when the timer first goes
   * active. {@code DriverStation.getMatchTime()} returns -1 off-FMS, which would
   * otherwise leave the bar empty (no lights) during bench testing.
   */
  private static DoubleSupplier matchOrSelf(DoubleSupplier total) {
    final double[] start = {-1};
    return () -> {
      double matchTime = DriverStation.getMatchTime();
      if (matchTime >= 0) {
        start[0] = -1; // re-arm the fallback in case the match clock drops out
        return matchTime;
      }
      double now = Timer.getFPGATimestamp();
      if (start[0] < 0) {
        start[0] = now;
      }
      return Math.max(0, total.getAsDouble() - (now - start[0]));
    };
  }

  /**
   * Tracks the real FRC match clock. The bar spans {@link MatchTimings#autonSec}
   * during autonomous and {@link MatchTimings#teleopSec} otherwise, with
   * remaining from {@code DriverStation.getMatchTime()}. Off-FMS it falls back to
   * a self-started countdown so the bar still works on the bench.
   */
  public static LedPattern matchTimer() {
    return matchTimer(MatchTimings.standard());
  }

  /** {@link #matchTimer()} with custom period lengths. */
  public static LedPattern matchTimer(MatchTimings timings) {
    DoubleSupplier total =
        () -> DriverStation.isAutonomous() ? timings.autonSec : timings.teleopSec;
    return timer(total, matchOrSelf(total));
  }

  /** Auton-length timer tracking the match clock, with off-FMS fallback. */
  public static LedPattern autonTimer() {
    return autonTimer(MatchTimings.REBUILT_AUTON_SEC);
  }

  public static LedPattern autonTimer(double autonSec) {
    return timer(autonSec, matchOrSelf(() -> autonSec));
  }

  /** Teleop-length timer tracking the match clock, with off-FMS fallback. */
  public static LedPattern teleopTimer() {
    return teleopTimer(MatchTimings.standard().teleopSec);
  }

  public static LedPattern teleopTimer(double teleopSec) {
    return timer(teleopSec, matchOrSelf(() -> teleopSec));
  }
}
