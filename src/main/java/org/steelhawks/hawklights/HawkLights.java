package org.steelhawks.hawklights;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.ArrayList;
import java.util.List;

/**
 * Robot-side LED controller for the HawkLights system.
 *
 * <p>Robot code declares <i>what it wants</i> and the highest-priority active
 * request wins. The resolved request is published to the {@code /HawkLights}
 * NetworkTables table; the laptop bridge app forwards it to the Pico over serial.
 * This class never touches hardware -- it only publishes intent.
 *
 * <h2>Usage</h2>
 *
 * <pre>
 *   HawkLights leds = new HawkLights();
 *
 *   // idle look while nothing else is asking
 *   leds.setIdle(LedPattern.solid(Color.kBlue));
 *
 *   // bind triggers to patterns (Trigger comes from your subsystems / controllers)
 *   leds.bind(shooter.atSpeed(), LedPattern.flash(Color.kGreen, 120), LedPriority.HIGH);
 *
 *   // or use the command directly in a Trigger binding
 *   intake.hasNote().whileTrue(leds.show(LedPattern.solid(Color.kOrange), LedPriority.DEFAULT));
 * </pre>
 *
 * <p>When no request is active the table's {@code active} flag is set false and
 * the bridge lets the Pico fall back to its own driver-station-state patterns.
 */
public class HawkLights extends SubsystemBase {

  /** Default NetworkTables table name; matches the bridge app. */
  public static final String DEFAULT_TABLE = "HawkLights";

  private final IntegerPublisher statePub;
  private final IntegerPublisher rPub;
  private final IntegerPublisher gPub;
  private final IntegerPublisher bPub;
  private final IntegerPublisher intervalPub;
  private final BooleanPublisher activePub;

  private final List<Request> requests = new ArrayList<>();
  private Request idle = null;

  public HawkLights() {
    this(NetworkTableInstance.getDefault().getTable(DEFAULT_TABLE));
  }

  public HawkLights(NetworkTable table) {
    statePub = table.getIntegerTopic("state").publish();
    rPub = table.getIntegerTopic("r").publish();
    gPub = table.getIntegerTopic("g").publish();
    bPub = table.getIntegerTopic("b").publish();
    intervalPub = table.getIntegerTopic("intervalMs").publish();
    activePub = table.getBooleanTopic("active").publish();
    activePub.set(false);
  }

  // ---- Trigger / command API ------------------------------------------------

  /**
   * Returns a command that, while scheduled, requests {@code pattern} at
   * {@code priority}. Bind it with {@code trigger.whileTrue(...)}. The command
   * runs while disabled (LEDs are status indication) and intentionally requires
   * no subsystem, so several requests can be active at once and priority decides.
   */
  public Command show(LedPattern pattern, LedPriority priority) {
    return show(pattern, priority.level);
  }

  /** {@link #show(LedPattern, LedPriority)} with a raw integer priority. */
  public Command show(LedPattern pattern, int priority) {
    Request req = new Request(pattern, priority);
    return Commands.startEnd(() -> requests.add(req), () -> requests.remove(req))
        .ignoringDisable(true)
        .withName("HawkLights:" + pattern.state());
  }

  /** {@link #show(LedPattern, LedPriority)} at {@link LedPriority#DEFAULT}. */
  public Command show(LedPattern pattern) {
    return show(pattern, LedPriority.DEFAULT);
  }

  /** Convenience: {@code trigger.whileTrue(show(pattern, priority))}. */
  public void bind(Trigger trigger, LedPattern pattern, LedPriority priority) {
    trigger.whileTrue(show(pattern, priority));
  }

  public void bind(Trigger trigger, LedPattern pattern) {
    bind(trigger, pattern, LedPriority.DEFAULT);
  }

  // ---- Idle baseline --------------------------------------------------------

  /** Pattern shown when no other request is active. Pass {@code null} to clear. */
  public void setIdle(LedPattern pattern) {
    idle = pattern == null ? null : new Request(pattern, LedPriority.IDLE.level);
  }

  public void clearIdle() {
    idle = null;
  }

  // ---- Resolution + publishing ---------------------------------------------

  @Override
  public void periodic() {
    Request best = idle;
    for (Request r : requests) {
      if (best == null || r.priority >= best.priority) {
        best = r;
      }
    }
    if (best == null) {
      activePub.set(false);
      return;
    }
    LedPattern p = best.pattern;
    statePub.set(p.state().value());
    rPub.set(p.color().red);
    gPub.set(p.color().green);
    bPub.set(p.color().blue);
    intervalPub.set(p.intervalMs());
    activePub.set(true);
  }

  private static final class Request {
    final LedPattern pattern;
    final int priority;

    Request(LedPattern pattern, int priority) {
      this.pattern = pattern;
      this.priority = priority;
    }
  }
}
