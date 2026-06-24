package org.hawklights.leds;

/**
 * Relative importance of an LED request. When several requests are active at
 * once, the highest priority wins (ties go to whichever became active last).
 *
 * <p>These are just convenient named levels; any {@code int} also works via the
 * {@code int}-typed overloads on {@link HawkLights}.
 */
public enum LedPriority {
  /** Background / idle indication (alliance color, "robot on", etc.). */
  IDLE(-1000),
  LOW(0),
  DEFAULT(10),
  HIGH(20),
  /** Safety / driver-critical signals (e.g. estop, low battery). */
  CRITICAL(30);

  public final int level;

  LedPriority(int level) {
    this.level = level;
  }
}
