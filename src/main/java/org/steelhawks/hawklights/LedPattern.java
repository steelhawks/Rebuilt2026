package org.hawklights.leds;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * An immutable LED request: which pattern, what color, and (for time-based
 * patterns) the interval in milliseconds. Build them with the static factories.
 *
 * <pre>
 *   LedPattern.solid(Color.kGreen)
 *   LedPattern.flash(new Color8Bit(255, 0, 0), 200)
 *   LedPattern.rainbow()
 * </pre>
 */
public record LedPattern(LedState state, Color8Bit color, int intervalMs) {

  private static final Color8Bit OFF = new Color8Bit(0, 0, 0);
  private static final int DEFAULT_INTERVAL_MS = 250;

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

  public static LedPattern flash(Color color) {
    return flash(new Color8Bit(color));
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

  /** Match-timer bar; color is ignored by the firmware. */
  public static LedPattern timer() {
    return new LedPattern(LedState.TIMER, OFF, 0);
  }
}
