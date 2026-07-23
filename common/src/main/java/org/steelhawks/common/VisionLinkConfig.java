package org.steelhawks.common;

import java.util.Arrays;
import java.util.Collections;
import java.util.HashSet;
import java.util.Objects;
import java.util.Set;

/**
 * Single source of truth for the config the RIO and the Orange Pi vision service
 * must agree on: the field/alliance tag sets and a version counter, hashed into
 * {@link #CONFIG_HASH}.
 *
 * <p>Both sides compile this same class (RIO root project and {@code :pi-service}
 * both depend on {@code :common}), so the hash is computed from one definition
 * and can only ever differ due to a genuine <em>deploy skew</em> - an out-of-date
 * jar on one side - which is exactly what the hash check exists to catch.
 *
 * <p>Camera extrinsics and stddev/rejection tuning are intentionally NOT here:
 * those are owned independently by the Pi.
 */
public final class VisionLinkConfig {

    private VisionLinkConfig() {}

    /**
     * Bump whenever the wire contract or the tag sets below change in a way the
     * Pi must know about. Forces a {@link #CONFIG_HASH} change so a half-updated
     * deploy is caught.
     */
    public static final int CONFIG_VERSION = 1;

    public static final int[] BLUE_TAGS = {
        17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32
    };

    public static final int[] BLUE_HUB_ONLY = {
        18, 19, 20, 21, 24, 25, 26, 27
    };

    public static final int[] RED_TAGS = {
        1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16
    };

    public static final int[] RED_HUB_ONLY = {
        2, 3, 4, 5, 8, 9, 10, 11
    };

    public static final int[] ALL_ALLOWED_TAGS = {
        0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16,
        17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32
    };

    public static final Set<Integer> HUB_TAG_IDS;

    static {
        Set<Integer> hub = new HashSet<>();
        for (int id : BLUE_HUB_ONLY) hub.add(id);
        for (int id : RED_HUB_ONLY) hub.add(id);
        HUB_TAG_IDS = Collections.unmodifiableSet(hub);
    }

    /** Fingerprint of the config both ends must agree on. */
    public static final long CONFIG_HASH = computeConfigHash();

    private static long computeConfigHash() {
        return ((long) Objects.hash(
                CONFIG_VERSION,
                Arrays.hashCode(BLUE_TAGS),
                Arrays.hashCode(RED_TAGS),
                Arrays.hashCode(BLUE_HUB_ONLY),
                Arrays.hashCode(RED_HUB_ONLY),
                HUB_TAG_IDS.hashCode()))
            & 0xFFFFFFFFL;
    }
}
