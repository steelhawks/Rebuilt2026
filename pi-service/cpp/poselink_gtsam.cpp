// JNI glue for the RIO<->Pi GTSAM pose estimator. All logic lives in
// poselink::PoseGraph; this file only marshals doubles across the boundary.
// Signatures match org.steelhawks.pi.gtsam.NativePoseEstimator.

#include <jni.h>

#include <gtsam/config.h>

#include <string>

#include "pose_graph.h"

using poselink::PoseGraph;

namespace {
    inline PoseGraph *asGraph(jlong h) { return reinterpret_cast<PoseGraph *>(h); }
} // namespace

extern "C" {
JNIEXPORT jstring JNICALL

Java_org_steelhawks_pi_gtsam_NativePoseEstimator_nativeVersion(JNIEnv *env, jclass) {
    std::string v = std::string("poselink_gtsam 1.0 (GTSAM ") + GTSAM_VERSION_STRING + ")";
    return env->NewStringUTF(v.c_str());
}

JNIEXPORT jlong JNICALL

Java_org_steelhawks_pi_gtsam_NativePoseEstimator_nativeCreate(JNIEnv *, jclass,
                                                              jdouble lagSeconds) {
    return reinterpret_cast<jlong>(new PoseGraph(lagSeconds));
}

JNIEXPORT void JNICALL

Java_org_steelhawks_pi_gtsam_NativePoseEstimator_nativeReset(
    JNIEnv *, jclass, jlong h, jdouble x, jdouble y, jdouble theta,
    jdouble covXX, jdouble covYY, jdouble covTheta) {
    if (auto *g = asGraph(h)) {
        g->reset(gtsam::Pose2(x, y, theta), gtsam::Vector3(covXX, covYY, covTheta));
    }
}

JNIEXPORT void JNICALL

Java_org_steelhawks_pi_gtsam_NativePoseEstimator_nativeAddOdometry(
    JNIEnv *, jclass, jlong h, jdouble t, jdouble dx, jdouble dy, jdouble dtheta,
    jdouble covXX, jdouble covYY, jdouble covTheta) {
    if (auto *g = asGraph(h)) {
        g->addOdometry(t, gtsam::Pose2(dx, dy, dtheta), gtsam::Vector3(covXX, covYY, covTheta));
    }
}

JNIEXPORT void JNICALL

Java_org_steelhawks_pi_gtsam_NativePoseEstimator_nativeAddVision(
    JNIEnv *, jclass, jlong h, jdouble t, jdouble x, jdouble y, jdouble theta,
    jdouble covXX, jdouble covYY, jdouble covTheta) {
    if (auto *g = asGraph(h)) {
        g->addVision(t, gtsam::Pose2(x, y, theta), gtsam::Vector3(covXX, covYY, covTheta));
    }
}

JNIEXPORT jint JNICALL

Java_org_steelhawks_pi_gtsam_NativePoseEstimator_nativeUpdate(JNIEnv *, jclass, jlong h) {
    auto *g = asGraph(h);
    return g ? g->update() : poselink::STATUS_NOT_INITIALIZED;
}

JNIEXPORT jdoubleArray JNICALL

Java_org_steelhawks_pi_gtsam_NativePoseEstimator_nativeGetResult(JNIEnv *env, jclass,
                                                                 jlong h) {
    jdoubleArray out = env->NewDoubleArray(10);
    auto *g = asGraph(h);
    if (!g) return out;

    gtsam::Pose2 p = g->pose();
    gtsam::Vector3 m = g->marginal();
    jdouble buf[10] = {
        p.x(), p.y(), p.theta(),
        m(0), m(1), m(2),
        static_cast<jdouble>(g->nodeCount()),
        static_cast<jdouble>(g->factorCount()),
        static_cast<jdouble>(g->status()),
        static_cast<jdouble>(g->timeJumps())
    };
    env->SetDoubleArrayRegion(out, 0, 10, buf);
    return out;
}

JNIEXPORT void JNICALL

Java_org_steelhawks_pi_gtsam_NativePoseEstimator_nativeDestroy(JNIEnv *, jclass, jlong h) {
    delete asGraph(h);
}
} // extern "C"
