// Standalone sanity checks for poselink::PoseGraph. No test framework - builds as
// a plain executable, prints PASS/FAIL per case, exits nonzero on any failure.
//
//   cmake --build cmake-build-debug --target pose_graph_test
//   ./cmake-build-debug/pose_graph_test

#include "pose_graph.h"

#include <cmath>
#include <cstdio>

using gtsam::Pose2;
using gtsam::Vector3;
using poselink::PoseGraph;

namespace {
    int g_failures = 0;

    void check(bool cond, const char *msg) {
        std::printf("  [%s] %s\n", cond ? "PASS" : "FAIL", msg);
        if (!cond) ++g_failures;
    }

    bool near(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

    Vector3 var(double a, double b, double c) { return Vector3(a, b, c); }

    // Drive N forward steps of `step` metres starting at t0, updating each time.
    double driveForward(PoseGraph &g, double t0, int n, double step) {
        double t = t0;
        for (int i = 0; i < n; ++i) {
            t += 0.02;
            g.addOdometry(t, Pose2(step, 0, 0), var(0.02, 0.02, 0.02));
            g.update();
        }
        return t;
    }
} // namespace

int main() {
    // 1. A reset with no motion reports exactly the reset pose.
    {
        std::printf("reset-only:\n");
        PoseGraph g(1.5);
        g.reset(Pose2(1.0, 2.0, 0.5), var(0.01, 0.01, 0.02));
        Pose2 p = g.pose();
        check(near(p.x(), 1.0, 1e-9) && near(p.y(), 2.0, 1e-9) && near(p.theta(), 0.5, 1e-9),
              "pose equals reset pose");
        check(g.status() == poselink::STATUS_OK, "status OK after reset");
    }

    // 2. Pure odometry integrates exactly (no conflicting measurement -> the MAP of
    //    an open chain is just the dead-reckoned pose). First sample only anchors,
    //    so 12 samples apply 11 deltas.
    {
        std::printf("pure-odometry:\n");
        PoseGraph g(1.5);
        g.reset(Pose2(1.0, 2.0, 0.0), var(0.01, 0.01, 0.01));
        driveForward(g, 0.0, 12, 0.1);
        Pose2 p = g.pose();
        check(near(p.x(), 1.0 + 11 * 0.1, 1e-2), "x integrated to ~2.1");
        check(near(p.y(), 2.0, 1e-6) && near(p.theta(), 0.0, 1e-6), "y/theta unchanged");
    }

    // 3. A tag that disagrees with odometry pulls the estimate toward the tag.
    {
        std::printf("tag-correction:\n");
        double xPure, xTagged;
        {
            PoseGraph g(1.5);
            g.reset(Pose2(0, 0, 0), var(0.05, 0.05, 0.05));
            driveForward(g, 0.0, 16, 0.1);
            xPure = g.pose().x();
        }
        {
            PoseGraph g(1.5);
            g.reset(Pose2(0, 0, 0), var(0.05, 0.05, 0.05));
            // early odom, then a tight tag at t=0.06 saying we were further back,
            // then keep driving so the tagged node commits.
            double t = 0.0;
            for (int i = 0; i < 3; ++i) {
                t += 0.02;
                g.addOdometry(t, Pose2(0.1, 0, 0), var(0.02, 0.02, 0.02));
                g.update();
            }
            g.addVision(0.06, Pose2(-0.3, 0, 0), var(1e-4, 1e-4, 1e-4));
            g.update();
            driveForward(g, t, 13, 0.1);
            xTagged = g.pose().x();
        }
        check(xTagged < xPure - 0.05, "tagged estimate pulled back from pure odom");
    }

    // 4. A tag whose timestamp falls between nodes exercises the splice path; it
    //    must be accepted and leave a sane, finite estimate.
    {
        std::printf("mid-interval splice:\n");
        PoseGraph g(1.5);
        g.reset(Pose2(0, 0, 0), var(0.05, 0.05, 0.05));
        double t = 0.0;
        for (int i = 0; i < 4; ++i) {
            t += 0.02;
            g.addOdometry(t, Pose2(0.1, 0, 0), var(0.02, 0.02, 0.02));
            g.update();
        }
        g.addVision(0.05, Pose2(0.15, 0.0, 0.0), var(1e-3, 1e-3, 1e-3)); // between 0.04 and 0.06
        driveForward(g, t, 12, 0.1);
        Pose2 p = g.pose();
        check(std::isfinite(p.x()) && std::isfinite(p.y()) && std::isfinite(p.theta()),
              "estimate finite after splice");
        check(g.status() == poselink::STATUS_OK, "status OK after splice");
    }

    std::printf("\n%s (%d failure%s)\n", g_failures == 0 ? "ALL PASS" : "FAILED",
                g_failures, g_failures == 1 ? "" : "s");
    return g_failures == 0 ? 0 : 1;
}
