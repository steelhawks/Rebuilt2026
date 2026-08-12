// Standalone sanity checks for poselink::PoseGraph. No test framework - builds as
// a plain executable, prints PASS/FAIL per case, exits nonzero on any failure.
//
//   cmake --build cmake-build-debug --target pose_graph_test
//   ./cmake-build-debug/pose_graph_test

#include "pose_graph.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <utility>
#include <vector>

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

    // ---- sustained-tracking simulation -------------------------------------
    //
    // Ground truth is a constant-curvature arc: 1 m/s along a 2 m radius, so the
    // robot both translates and rotates the whole time. Odometry deltas and tag
    // observations are both read off this same truth with no noise, so the graph
    // is perfectly consistent and the exact answer is available. Rates match the
    // robot: odometry 50 Hz, each camera 30 Hz, tags 100 ms stale by the time
    // they are handed over (measured skew in the 2026-08-10 logs was 69-112 ms).

    constexpr double kOdomDt = 0.02;
    constexpr double kVisionLatency = 0.10;
    constexpr double kOmega = 0.5;   // rad/s
    constexpr double kRadius = 2.0;  // m

    Pose2 truthAt(double t) {
        double th = kOmega * t;
        return Pose2(kRadius * std::sin(th), kRadius * (1.0 - std::cos(th)), th);
    }

    struct Sim {
        double maxPosErr = 0.0;
        double maxHeadingErrDeg = 0.0;
        double finalPosErr = 0.0;
        double sigmaAtMaxErr = 0.0;
        bool finite = true;
    };

    // odomScale/odomHeadingBias inject a systematic odometry error, which is the
    // situation on the real robot: vision is self-consistent (cameras agree to
    // 2 cm) so any lasting disagreement is between vision and odometry. odomVar
    // is what the graph is TOLD odometry is worth.
    // Mirrors PiVisionConstants.ODOM_VARIANCE_LINEAR / _ANGULAR. Keep in sync -
    // the C++ side only ever sees what Java hands it, so this is the one place
    // the values are asserted to behave.
    const Vector3 kOdomVar(2e-3, 2e-3, 1e-6);

    Sim simulate(int nCameras, double seconds,
                 double odomScale = 1.0, double odomHeadingBias = 0.0,
                 Vector3 odomVar = kOdomVar) {
        Sim out;
        PoseGraph g(1.5);
        g.reset(truthAt(0.0), var(0.01, 0.01, 0.02));

        // Stagger the cameras so their frames land at different points inside the
        // odometry intervals - that is what drives the splice path.
        std::vector<double> nextFrame(nCameras);
        for (int c = 0; c < nCameras; ++c) {
            nextFrame[c] = kVisionLatency + (c / static_cast<double>(nCameras)) / 30.0;
        }

        double t = 0.0;
        while (t < seconds) {
            double prev = t;
            t += kOdomDt;
            Pose2 d = truthAt(prev).between(truthAt(t));
            Pose2 measured(d.x() * odomScale, d.y() * odomScale,
                           d.theta() + odomHeadingBias * kOdomDt);
            g.addOdometry(t, measured, odomVar);

            for (int c = 0; c < nCameras; ++c) {
                while (nextFrame[c] <= t) {
                    double capture = nextFrame[c] - kVisionLatency;
                    if (capture > 0.0) {
                        g.addVision(capture, truthAt(capture), var(2.5e-3, 2.5e-3, 4e-4));
                    }
                    nextFrame[c] += 1.0 / 30.0;
                }
            }

            g.update();

            Pose2 p = g.pose();
            if (!std::isfinite(p.x()) || !std::isfinite(p.y()) || !std::isfinite(p.theta())) {
                out.finite = false;
                return out;
            }
            Pose2 truth = truthAt(t);
            double e = std::hypot(p.x() - truth.x(), p.y() - truth.y());
            double dth = std::fabs(std::remainder(p.theta() - truth.theta(), 2.0 * M_PI));
            if (e > out.maxPosErr) {
                out.maxPosErr = e;
                Vector3 m = g.marginal();
                out.sigmaAtMaxErr = std::sqrt(std::fabs(m(0)) + std::fabs(m(1)));
            }
            out.maxHeadingErrDeg = std::max(out.maxHeadingErrDeg, dth * 180.0 / M_PI);
            out.finalPosErr = e;
        }
        return out;
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

    // 5. The regime that actually fails on the robot: sustained driving with
    //    several cameras streaming, for tens of seconds.
    //
    //    Cases 1-4 are single-shot happy paths, which is why they kept passing
    //    while the field logs showed the estimate sitting metres from cameras
    //    that agreed with each other to 2 cm. Here odometry AND vision are
    //    derived from the same ground truth with no noise at all, so the graph is
    //    perfectly self-consistent and the only correct answer is the truth.
    //    Whatever error comes out is the estimator's own.
    for (int nCams : {1, 3, 5}) {
        std::printf("sustained %d-camera tracking:\n", nCams);
        Sim s = simulate(nCams, 30.0);
        std::printf("    max err %.4f m / %.2f deg, final %.4f m,"
                    " reported sigma %.4f m at max err\n",
                    s.maxPosErr, s.maxHeadingErrDeg, s.finalPosErr, s.sigmaAtMaxErr);
        check(s.finite, "estimate stayed finite");
        check(s.maxPosErr < 0.10, "max position error under 10 cm");
        check(s.maxHeadingErrDeg < 2.0, "max heading error under 2 deg");
        // An estimator may be wrong; it may not be wrong AND confident. If the
        // reported sigma does not cover the actual error, every consumer
        // downstream - stddev tuning, quality gating - is working off a number
        // that means nothing.
        check(s.sigmaAtMaxErr * 3.0 > s.maxPosErr,
              "reported 3-sigma covers actual error");
    }

    // 5b. Can vision correct odometry that is wrong? This is the real question:
    //     on the robot the cameras agree with each other to 2 cm, so any lasting
    //     disagreement is vision-versus-odometry, and the graph believes whichever
    //     it was told to. ODOM_VARIANCE_LINEAR=1e-5 asserts 3.2 mm per 20 ms step;
    //     ODOM_VARIANCE_ANGULAR=1e-6 asserts 0.057 deg. If real odometry is worse
    //     than that - any wheel slip, any heading offset - vision mathematically
    //     cannot win the argument.
    {
        // The old ODOM_VARIANCE_ANGULAR, kept as a regression witness: if someone
        // tightens it back, this prints the cost in metres rather than leaving it
        // to be rediscovered on the field.
        std::printf("odometry error, 5 cameras - variance sweep:\n");
        std::printf("    lin      ang     2%%scale   headbias  maxdeg\n");
        for (double lin : {1e-5, 1e-4, 1e-3}) {
            for (double ang : {1e-6, 1e-5, 1e-4}) {
                Vector3 v(lin, lin, ang);
                Sim s = simulate(5, 30.0, 1.02, 0.0, v);
                Sim h = simulate(5, 30.0, 1.0, 0.02, v);
                std::printf("   %7.0e  %7.0e  %7.3f   %7.3f   %5.1f\n",
                            lin, ang, s.finalPosErr, h.finalPosErr, h.maxHeadingErrDeg);
            }
        }
        Sim scale = simulate(5, 30.0, 1.02);
        Sim head = simulate(5, 30.0, 1.0, 0.02);
        check(scale.finalPosErr < 0.15, "vision corrects a 2% odometry scale error");
        check(head.finalPosErr < 0.15, "vision corrects an odometry heading bias");
    }

    // 6. Two observations landing on the same node must both reach the graph.
    //    Node holds a single priorPose/priorVar, so attach() overwrites; with
    //    several cameras at 30 Hz, ~10% of frames collide inside spliceEps.
    {
        std::printf("simultaneous observations from two cameras:\n");
        // Both tags say x=0.5. Odometry says x=1.0. One tag should pull the
        // estimate toward 0.5; two identical tags should pull at least as hard.
        auto run = [](int nObs) {
            PoseGraph g(1.5);
            g.reset(Pose2(0, 0, 0), var(0.05, 0.05, 0.05));
            double t = 0.0;
            for (int i = 0; i < 5; ++i) {
                t += 0.02;
                g.addOdometry(t, Pose2(0.2, 0, 0), var(1e-4, 1e-4, 1e-4));
                g.update();
            }
            for (int k = 0; k < nObs; ++k) {
                g.addVision(0.051, Pose2(0.5, 0, 0), var(1e-3, 1e-3, 1e-3));
            }
            driveForward(g, t, 20, 0.0);
            return g.pose().x();
        };
        double one = run(1);
        double two = run(2);
        std::printf("    1 observation -> x=%.4f,  2 identical -> x=%.4f\n", one, two);
        check(std::fabs(two - one) > 1e-6,
              "a second simultaneous observation changes the estimate");
    }

    std::printf("\n%s (%d failure%s)\n", g_failures == 0 ? "ALL PASS" : "FAILED",
                g_failures, g_failures == 1 ? "" : "s");
    return g_failures == 0 ? 0 : 1;
}
