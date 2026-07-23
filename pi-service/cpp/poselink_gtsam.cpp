// JNI shim for the RIO<->Pi GTSAM pose estimator.
//
// SLICE 1: STUB. This validates the JNI surface + library load only. It stores
// the last reset/measurement so getResult() echoes something sane, but builds no
// factor graph. Slice 2 replaces the guts with a gtsam::IncrementalFixedLagSmoother
// (SE(2), odom BetweenFactor + gyro rotation, vision PriorFactor with
// splice-or-attach) while keeping every signature below unchanged.

#include <jni.h>
#include <cstring>
#include <string>

namespace {

// Status codes mirror NativePoseEstimator.STATUS_*.
constexpr int STATUS_OK = 0;
constexpr int STATUS_NOT_INITIALIZED = 1;
// constexpr int STATUS_INDETERMINATE = 2;  // used in slice 2

struct EstimatorStub {
    double lagSeconds = 1.5;
    bool anchored = false;
    double x = 0.0, y = 0.0, theta = 0.0;
    double covXX = 0.0, covYY = 0.0, covTheta = 0.0;
    int nodeCount = 0;
    int factorCount = 0;
};

inline EstimatorStub* asEstimator(jlong handle) {
    return reinterpret_cast<EstimatorStub*>(handle);
}

}  // namespace

extern "C" {

JNIEXPORT jstring JNICALL
Java_org_steelhawks_pi_gtsam_NativePoseEstimator_nativeVersion(JNIEnv* env, jclass) {
    // Slice 2 will append GTSAM_VERSION_STRING here.
    return env->NewStringUTF("poselink_gtsam stub 0.1 (no GTSAM linked yet)");
}

JNIEXPORT jlong JNICALL
Java_org_steelhawks_pi_gtsam_NativePoseEstimator_nativeCreate(JNIEnv*, jclass,
                                                              jdouble lagSeconds) {
    auto* e = new EstimatorStub();
    e->lagSeconds = lagSeconds;
    return reinterpret_cast<jlong>(e);
}

JNIEXPORT void JNICALL
Java_org_steelhawks_pi_gtsam_NativePoseEstimator_nativeReset(
    JNIEnv*, jclass, jlong h, jdouble x, jdouble y, jdouble theta,
    jdouble covXX, jdouble covYY, jdouble covTheta) {
    auto* e = asEstimator(h);
    if (!e) return;
    e->anchored = true;
    e->x = x; e->y = y; e->theta = theta;
    e->covXX = covXX; e->covYY = covYY; e->covTheta = covTheta;
    e->nodeCount = 1;
    e->factorCount = 1;
}

JNIEXPORT void JNICALL
Java_org_steelhawks_pi_gtsam_NativePoseEstimator_nativeAddOdometry(
    JNIEnv*, jclass, jlong h, jdouble /*t*/, jdouble dx, jdouble dy, jdouble dtheta,
    jdouble /*covXX*/, jdouble /*covYY*/, jdouble /*covTheta*/) {
    auto* e = asEstimator(h);
    if (!e || !e->anchored) return;
    e->x += dx; e->y += dy; e->theta += dtheta;
    e->nodeCount += 1;
    e->factorCount += 1;
}

JNIEXPORT void JNICALL
Java_org_steelhawks_pi_gtsam_NativePoseEstimator_nativeAddVision(
    JNIEnv*, jclass, jlong h, jdouble /*t*/, jdouble x, jdouble y, jdouble theta,
    jdouble covXX, jdouble covYY, jdouble covTheta) {
    auto* e = asEstimator(h);
    if (!e) return;
    e->anchored = true;
    e->x = x; e->y = y; e->theta = theta;
    e->covXX = covXX; e->covYY = covYY; e->covTheta = covTheta;
    e->factorCount += 1;
}

JNIEXPORT jint JNICALL
Java_org_steelhawks_pi_gtsam_NativePoseEstimator_nativeUpdate(JNIEnv*, jclass, jlong h) {
    auto* e = asEstimator(h);
    if (!e) return STATUS_NOT_INITIALIZED;
    return e->anchored ? STATUS_OK : STATUS_NOT_INITIALIZED;
}

JNIEXPORT jdoubleArray JNICALL
Java_org_steelhawks_pi_gtsam_NativePoseEstimator_nativeGetResult(JNIEnv* env, jclass,
                                                                jlong h) {
    auto* e = asEstimator(h);
    jdoubleArray out = env->NewDoubleArray(9);
    if (!e) return out;
    jdouble buf[9] = {
        e->x, e->y, e->theta,
        e->covXX, e->covYY, e->covTheta,
        static_cast<jdouble>(e->nodeCount),
        static_cast<jdouble>(e->factorCount),
        static_cast<jdouble>(e->anchored ? STATUS_OK : STATUS_NOT_INITIALIZED)
    };
    env->SetDoubleArrayRegion(out, 0, 9, buf);
    return out;
}

JNIEXPORT void JNICALL
Java_org_steelhawks_pi_gtsam_NativePoseEstimator_nativeDestroy(JNIEnv*, jclass, jlong h) {
    delete asEstimator(h);
}

}  // extern "C"
