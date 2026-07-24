package org.steelhawks.pi.vision;

/** A vision observation that passed filtering, with GTSAM-ready variances. */
public record AcceptedObservation(
    double timestamp,
    double x,
    double y,
    double theta,
    double varX,
    double varY,
    double varTheta) {}
