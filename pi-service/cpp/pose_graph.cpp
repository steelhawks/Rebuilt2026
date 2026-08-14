#include "pose_graph.h"

#include <gtsam/linear/linearExceptions.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace poselink {
    using gtsam::BetweenFactor;
    using gtsam::Matrix;
    using gtsam::Pose2;
    using gtsam::Vector3;

    PoseGraph::PoseGraph(double lagSeconds)
        : lag_(lagSeconds), smoother_(lagSeconds, isamParams_) {
        frontierMargin_ = Vector3::Zero();
    }

    gtsam::SharedNoiseModel PoseGraph::noise(const Vector3 &var) {
        return gtsam::noiseModel::Diagonal::Variances(var);
    }

    Pose2 PoseGraph::interpolate(const Pose2 &delta, double f) {
        return Pose2::Expmap(f * Pose2::Logmap(delta));
    }

    void PoseGraph::rebuild() {
        smoother_ = gtsam::IncrementalFixedLagSmoother(lag_, isamParams_);
        nodes_.clear();
        pending_.clear();
        extraPriors_.clear();
        nextKey_ = 0;
        factorCount_ = 0;
        frontierMargin_ = Vector3::Zero();
    }

    void PoseGraph::reset(const Pose2 &pose, const Vector3 &variances) {
        rebuild();
        haveAnchor_ = true;
        anchorPose_ = pose;
        anchorVar_ = variances;
        status_ = STATUS_OK;
    }

    void PoseGraph::reanchorHere() {
        // pose() and marginal() both walk the staged nodes, so they must be read
        // before rebuild() throws that list away.
        Pose2 here = pose();
        Vector3 m = marginal();
        Vector3 v(std::max(std::fabs(m(0)), kMinReanchorLinearVar),
                  std::max(std::fabs(m(1)), kMinReanchorLinearVar),
                  std::max(std::fabs(m(2)), kMinReanchorAngularVar));
        reset(here, v);
        ++timeJumps_;
    }

    bool PoseGraph::ensureAnchorNode(double t) {
        if (!nodes_.empty() || !haveAnchor_) return false;
        Node n;
        n.t = t;
        n.value = anchorPose_;
        n.delta = Pose2();
        n.odomVar = Vector3::Zero();
        n.isAnchor = true;
        n.priors.push_back({anchorPose_, anchorVar_});
        nodes_.push_back(n);
        return true;
    }

    void PoseGraph::addOdometry(double t, const Pose2 &delta, const Vector3 &variances) {
        // The RIO's clock restarts at zero on every code deploy, and this graph
        // stages nodes by measuring them against the newest timestamp it holds.
        // A backwards jump strands every staged node in the future, so
        // commitAgedNodes() breaks on the first of them and never commits another
        // node for the rest of the process's life: odometry piles up uncommitted,
        // accepted tags attach to nodes the smoother will never see, and the output
        // free-runs from a frozen anchor while still reporting STATUS_OK. Every Pi
        // log from 2026-08-13 that outlived a redeploy shows it - Nodes flat from
        // the instant the RIO went quiet, Factors still climbing, vision-vs-fused
        // error walking out to 17 cm with the quality score decaying to 0.05.
        //
        // Starting the graph over on the new time base costs nothing real: the robot
        // does not move while its own code is rebooting, so the pose we are holding
        // is still the right answer and is what we re-anchor on.
        if (!nodes_.empty() && t < nodes_.back().t - kBackwardsJumpTolerance) {
            reanchorHere();
        }

        if (nodes_.empty()) {
            // The first sample after an anchor only fixes the anchor's timestamp; it has
            // no previous node, so its delta is dropped.
            if (ensureAnchorNode(t)) return;
            if (!haveAnchor_) return; // no reset and no tag yet
        }

        Node n;
        n.t = t;
        n.delta = delta;
        n.odomVar = variances;
        n.value = nodes_.back().value.compose(delta);
        nodes_.push_back(n);

        std::vector<PendingVision> stillPending;
        for (const auto &v: pending_) {
            if (v.t <= nodes_.back().t) {
                placeVision(v);
            } else {
                stillPending.push_back(v);
            }
        }
        pending_.swap(stillPending);
    }

    void PoseGraph::addVision(double t, const Pose2 &pose, const Vector3 &variances) {
        if (nodes_.empty()) {
            if (haveAnchor_) {
                ensureAnchorNode(t);
            } else {
                // Tag auto-localization: this observation becomes the anchor.
                haveAnchor_ = true;
                anchorPose_ = pose;
                anchorVar_ = variances;
                ensureAnchorNode(t);
                status_ = STATUS_OK;
                return;
            }
        }

        PendingVision v{t, pose, variances};
        if (t > nodes_.back().t) {
            pending_.push_back(v); // its bracketing odom hasn't arrived yet
        } else {
            placeVision(v);
        }
    }

    void PoseGraph::placeVision(const PendingVision &v) {
        if (nodes_.empty()) return;

        int nearest = 0;
        double best = std::numeric_limits<double>::max();
        for (int i = 0; i < static_cast<int>(nodes_.size()); ++i) {
            double d = std::fabs(nodes_[i].t - v.t);
            if (d < best) {
                best = d;
                nearest = i;
            }
        }

        auto attach = [&](int idx) {
            Node &n = nodes_[idx];
            if (n.committed) {
                extraPriors_.push_back({n.key, v.pose, v.var});
            } else {
                // Append. Two cameras seeing the same instant are two independent
                // measurements and both belong in the graph.
                n.priors.push_back({v.pose, v.var});
            }
        };

        if (best <= spliceEps_) {
            attach(nearest);
            return;
        }

        int i = 0;
        while (i < static_cast<int>(nodes_.size()) && nodes_[i].t < v.t) ++i;
        if (i == 0) {
            // before the anchor
            attach(0);
            return;
        }
        if (i < static_cast<int>(nodes_.size()) && !nodes_[i].committed) {
            double t0 = nodes_[i - 1].t;
            double t1 = nodes_[i].t;
            double f = (v.t - t0) / (t1 - t0);
            Pose2 d = nodes_[i].delta;
            Pose2 d1 = interpolate(d, f);

            Node mid;
            mid.t = v.t;
            mid.delta = d1;
            mid.odomVar = nodes_[i].odomVar * f;
            mid.value = nodes_[i - 1].value.compose(d1);
            mid.priors.push_back({v.pose, v.var});

            nodes_[i].delta = d1.between(d);
            nodes_[i].odomVar = nodes_[i].odomVar * (1.0 - f);
            nodes_.insert(nodes_.begin() + i, mid);
            return;
        }

        // Bracket already flushed to the smoother: too late to splice, attach nearest.
        attach(nearest);
    }

    void PoseGraph::commitAgedNodes() {
        if (nodes_.empty()) return;
        double newest = nodes_.back().t;

        gtsam::NonlinearFactorGraph newFactors;
        gtsam::Values newValues;
        gtsam::IncrementalFixedLagSmoother::KeyTimestampMap stamps;

        int prev = lastCommittedIndex();
        for (int i = 0; i < static_cast<int>(nodes_.size()); ++i) {
            Node &n = nodes_[i];
            if (n.committed) {
                prev = i;
                continue;
            }
            if (newest - n.t < stageWindow_) break; // still inside the staging window

            n.key = nextKey_++;
            newValues.insert(n.key, n.value);
            stamps[n.key] = n.t;

            if (n.isAnchor || prev < 0) {
                // No predecessor to chain from, so this node needs an absolute
                // constraint or it enters the graph unconstrained. Fall back to
                // the anchor pose if nothing else has landed on it - the previous
                // code read an uninitialised priorPose/priorVar here.
                if (n.priors.empty()) {
                    n.priors.push_back({anchorPose_, anchorVar_});
                }
            } else {
                newFactors.emplace_shared<BetweenFactor<Pose2> >(
                    nodes_[prev].key, n.key, n.delta, noise(n.odomVar));
                ++factorCount_;
            }
            for (const Prior &pr : n.priors) {
                newFactors.addPrior(n.key, pr.pose, noise(pr.var));
                ++factorCount_;
            }
            n.committed = true;
            prev = i;
        }

        for (const auto &ep: extraPriors_) {
            newFactors.addPrior(ep.key, ep.pose, noise(ep.var));
            ++factorCount_;
        }
        extraPriors_.clear();

        if (newFactors.empty() && newValues.empty()) return;
        smoother_.update(newFactors, newValues, stamps);
    }

    int PoseGraph::update() {
        if (!haveAnchor_) {
            status_ = STATUS_NOT_INITIALIZED;
            return status_;
        }
        try {
            commitAgedNodes();
            int last = lastCommittedIndex();
            if (last >= 0) {
                nodes_[last].value = smoother_.calculateEstimate<Pose2>(nodes_[last].key);
                frontierMargin_ = smoother_.marginalCovariance(nodes_[last].key).diagonal().head<3>();
                // The smoother holds the older committed nodes; we only chain from the
                // frontier, so drop the rest to keep this list bounded.
                if (last > 0) nodes_.erase(nodes_.begin(), nodes_.begin() + last);
            }
            status_ = STATUS_OK;
        } catch (const gtsam::IndeterminantLinearSystemException &) {
            status_ = STATUS_INDETERMINATE;
            haveAnchor_ = false; // wait for a fresh reset or tag before trusting output
            rebuild();
        }
        return status_;
    }

    int PoseGraph::lastCommittedIndex() const {
        for (int i = static_cast<int>(nodes_.size()) - 1; i >= 0; --i) {
            if (nodes_[i].committed) return i;
        }
        return -1;
    }

    Pose2 PoseGraph::pose() const {
        if (nodes_.empty()) return haveAnchor_ ? anchorPose_ : Pose2();
        int last = lastCommittedIndex();
        Pose2 p = (last >= 0) ? nodes_[last].value : nodes_[0].value;
        int start = (last >= 0) ? last + 1 : 1;
        for (int i = start; i < static_cast<int>(nodes_.size()); ++i) {
            p = p.compose(nodes_[i].delta);
        }
        return p;
    }

    Vector3 PoseGraph::marginal() const {
        int last = lastCommittedIndex();
        Vector3 m = (last >= 0) ? frontierMargin_ : anchorVar_;
        int start = (last >= 0) ? last + 1 : 1;
        for (int i = start; i < static_cast<int>(nodes_.size()); ++i) {
            m += nodes_[i].odomVar;
        }
        return m;
    }

    int PoseGraph::nodeCount() const { return static_cast<int>(nextKey_); }
    int PoseGraph::factorCount() const { return factorCount_; }
} // namespace poselink
