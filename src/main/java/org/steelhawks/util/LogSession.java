package org.steelhawks.util;

import java.security.SecureRandom;
import org.littletonrobotics.junction.Logger;

/**
 * A random identifier for one RIO boot, used to pair this run's logs with the
 * logs the Orange Pi wrote for the same run.
 *
 * <p>Nothing else can do this job. The Pi has no RTC, so its wall clock is
 * whatever it was when the image was built - the 2026-08-07 session produced a
 * Pi log named {@code akit_26-06-06_07-14-14.wpilog}, two months off. Both logs
 * also start their own timestamps at zero, so neither filenames nor timestamps
 * can tell you whether two files came from the same run. Trying to recover the
 * pairing after the fact by cross-correlating logged values does not work
 * either: it was attempted on the 2026-08-07 pair and returned no alignment.
 *
 * <p>So the RIO mints an id at boot, records it as log metadata, and ships it to
 * the Pi over the link ({@code RobotOdomInputs.session_id}). The Pi logs it every
 * cycle. {@code tools/pull_logs.py} reads both sides and pairs them by value.
 */
public final class LogSession {

    private LogSession() {}

    /** Metadata key holding {@link #idHex()} in both the RIO and Pi logs. */
    public static final String METADATA_KEY = "SessionId";

    private static final long ID = new SecureRandom().nextLong();
    private static final String ID_HEX = String.format("%016x", ID);

    /** The raw id, as sent over the link. */
    public static long id() {
        return ID;
    }

    /** The id as 16 lowercase hex digits - what appears in metadata and filenames. */
    public static String idHex() {
        return ID_HEX;
    }

    /** Record the id as log metadata. Must run before {@code Logger.start()}. */
    public static void recordMetadata() {
        Logger.recordMetadata(METADATA_KEY, ID_HEX);
    }
}
