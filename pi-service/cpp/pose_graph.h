#pragma once

#include <gtsam/geometry/Pose2.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

#include <vector>

namespace poselink {
    enum Status { STATUS_OK = 0, STATUS_NOT_INITIALIZED = 1, STATUS_INDETERMINATE = 2 };

    // SE(2) pose-graph over a GTSAM fixed-lag smoother. Wheel odometry + gyro come in
    // as BetweenFactors; each accepted AprilTag pose is an absolute PriorFactor. Tag
    // observations arrive late (capture + solve + network), so recent odom edges are
    // held in a staging window and only handed to the smoother once they're older
    // than the worst-case vision latency. A tag landing inside that window splits the
    // staged edge at its capture time; one already flushed to the smoother falls back
    // to attaching onto the nearest node. This keeps the splice entirely in our own
    // buffer, so GTSAM never needs a factor removed.
    class PoseGraph {
    public:
        explicit PoseGraph(double lagSeconds);

        void reset(const gtsam::Pose2 &pose, const gtsam::Vector3 &variances);

        void addOdometry(double t, const gtsam::Pose2 &delta, const gtsam::Vector3 &variances);

        void addVision(double t, const gtsam::Pose2 &pose, const gtsam::Vector3 &variances);

        int update();

        gtsam::Pose2 pose() const;

        gtsam::Vector3 marginal() const;

        int nodeCount() const;

        int factorCount() const;

        int status() const { return status_; }

    private:
        struct Node {
            double t;
            gtsam::Pose2 value; // best estimate; refreshed from the smoother once committed
            gtsam::Pose2 delta; // motion from the previous node (identity for the anchor)
            gtsam::Vector3 odomVar;
            bool isAnchor = false;
            bool hasPrior = false;
            gtsam::Pose2 priorPose;
            gtsam::Vector3 priorVar;
            bool committed = false;
            gtsam::Key key = 0;
        };

        struct PendingVision {
            double t;
            gtsam::Pose2 pose;
            gtsam::Vector3 var;
        };

        struct ExtraPrior {
            gtsam::Key key;
            gtsam::Pose2 pose;
            gtsam::Vector3 var;
        };

        void rebuild();

        bool ensureAnchorNode(double t); // materialize node 0 at the first timestamp we see
        void placeVision(const PendingVision &v);

        void commitAgedNodes();

        int lastCommittedIndex() const;

        static gtsam::SharedNoiseModel noise(const gtsam::Vector3 &var);

        static gtsam::Pose2 interpolate(const gtsam::Pose2 &delta, double f);

        double lag_;
        double stageWindow_ = 0.15; // hold odom edges this long so late tags can still splice
        double spliceEps_ = 0.005; // within this of a node -> attach instead of splice

        gtsam::ISAM2Params isamParams_;
        gtsam::IncrementalFixedLagSmoother smoother_;

        bool haveAnchor_ = false;
        gtsam::Pose2 anchorPose_;
        gtsam::Vector3 anchorVar_;

        std::vector<Node> nodes_; // time-ordered; committed nodes pruned past the lag
        std::vector<PendingVision> pending_; // tags not yet placeable (t beyond the newest node)
        std::vector<ExtraPrior> extraPriors_; // tags that landed on an already-committed node
        gtsam::Key nextKey_ = 0;

        gtsam::Vector3 frontierMargin_;
        int factorCount_ = 0;
        int status_ = STATUS_NOT_INITIALIZED;
    };
} // namespace poselink
