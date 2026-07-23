package org.steelhawks.pi.gtsam;

/**
 * JNI wrapper over the narrow C++ GTSAM shim ({@code libposelink_gtsam.so}).
 *
 * <p>The C++ side ({@code poselink::PoseGraph}) owns the
 * {@code IncrementalFixedLagSmoother} and all GTSAM types; this class is a thin
 * handle-based facade. The shim is "dumb": it takes already-accepted
 * {@code (Pose2, covariance)} measurements and returns pose + marginal
 * covariance. Whitelisting, rejection, and stddev weighting stay in Java.
 */
public final class NativePoseEstimator implements AutoCloseable {

    static {
        // libposelink_gtsam.so must be on java.library.path (set by the systemd
        // unit / launch script to the deploy dir).
        System.loadLibrary("poselink_gtsam");
    }

    /** Status codes returned by {@link #update()} / carried in {@link Result}. */
    public static final int STATUS_OK = 0;
    public static final int STATUS_NOT_INITIALIZED = 1;
    public static final int STATUS_INDETERMINATE = 2;

    private long handle;

    public NativePoseEstimator(double lagSeconds) {
        this.handle = nativeCreate(lagSeconds);
        if (handle == 0) {
            throw new IllegalStateException("Failed to create native GTSAM estimator");
        }
    }

    /** Version string of the linked shim + GTSAM. Cheap JNI-load smoke test. */
    public static String version() {
        return nativeVersion();
    }

    /** Rebuild the graph, anchoring node 0 at {@code pose} with a tight prior. */
    public void reset(double x, double y, double theta,
                      double covXX, double covYY, double covTheta) {
        nativeReset(handle, x, y, theta, covXX, covYY, covTheta);
    }

    /** Odometry delta between the previous node and time {@code t} (RIO clock). */
    public void addOdometry(double t, double dx, double dy, double dtheta,
                            double covXX, double covYY, double covTheta) {
        nativeAddOdometry(handle, t, dx, dy, dtheta, covXX, covYY, covTheta);
    }

    /**
     * Absolute pose measurement at time {@code t} (splice-or-attach: the shim
     * resolves {@code t} to an existing node within tolerance, else splices one).
     */
    public void addVisionMeasurement(double t, double x, double y, double theta,
                                     double covXX, double covYY, double covTheta) {
        nativeAddVision(handle, t, x, y, theta, covXX, covYY, covTheta);
    }

    /** Solve; returns a {@code STATUS_*} code. */
    public int update() {
        return nativeUpdate(handle);
    }

    /** Latest estimate + marginal + diagnostics. */
    public Result getResult() {
        double[] r = nativeGetResult(handle);
        return new Result(
            r[0], r[1], r[2],           // x, y, theta
            r[3], r[4], r[5],           // covXX, covYY, covTheta
            (int) r[6], (int) r[7],     // node count, factor count
            (int) r[8]);                // status
    }

    @Override
    public void close() {
        if (handle != 0) {
            nativeDestroy(handle);
            handle = 0;
        }
    }

    /** Immutable snapshot returned by {@link #getResult()}. */
    public record Result(
        double x, double y, double theta,
        double covXX, double covYY, double covTheta,
        int nodeCount, int factorCount, int status) {}

    // ---- native surface (see cpp/poselink_gtsam.cpp) ----
    private static native String nativeVersion();
    private static native long nativeCreate(double lagSeconds);
    private static native void nativeReset(long h, double x, double y, double theta,
        double covXX, double covYY, double covTheta);
    private static native void nativeAddOdometry(long h, double t, double dx, double dy,
        double dtheta, double covXX, double covYY, double covTheta);
    private static native void nativeAddVision(long h, double t, double x, double y,
        double theta, double covXX, double covYY, double covTheta);
    private static native int nativeUpdate(long h);
    private static native double[] nativeGetResult(long h);
    private static native void nativeDestroy(long h);
}
