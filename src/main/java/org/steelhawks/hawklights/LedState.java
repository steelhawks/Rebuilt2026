package org.steelhawks.hawklights;

/**
 * LED patterns the Pico knows how to render. The {@link #value()} ordinals must
 * stay in sync with the {@code LEDWantedState} enum in embedded/protocol.h.
 */
public enum LedState {
  SOLID(0),
  RAINBOW(1),
  FLASH(2),
  BOUNCE(3),
  FADE(4),
  TIMER(5),
  INDICATE_LEFT(6),
  INDICATE_RIGHT(7);

  private final int value;

  LedState(int value) {
    this.value = value;
  }

  /** Wire value sent over NetworkTables / serial. */
  public int value() {
    return value;
  }
}
