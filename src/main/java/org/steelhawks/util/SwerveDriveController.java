package org.steelhawks.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SwerveDriveController {

    private final PIDController xController;
    private final PIDController yController;
    private final ProfiledPIDController thetaController;
    private boolean firstRun = true;

    public SwerveDriveController(
        PIDController xController, PIDController yController, ProfiledPIDController thetaController) {
        this.xController = xController;
        this.yController = yController;
        this.thetaController = thetaController;
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public SwerveDriveController withLinearTolerance(double xyTolerance) {
        xController.setTolerance(xyTolerance);
        yController.setTolerance(xyTolerance);
        return this;
    }

    public SwerveDriveController withRotationalTolerance(double thetaTolerance) {
        thetaController.setTolerance(thetaTolerance);
        return this;
    }

    public PIDController getXController() {
        return xController;
    }

    public PIDController getYController() {
        return yController;
    }

    public ProfiledPIDController getThetaController() {
        return thetaController;
    }

    public void reset(Pose2d measurement) {
        xController.reset();
        yController.reset();
        thetaController.reset(measurement.getRotation().getRadians());
    }

    public ChassisSpeeds getOutput(Pose2d measurement, Pose2d setpoint) {
        if (firstRun) {
            reset(measurement);
            firstRun = false;
        }

        double xOutput = xController.calculate(measurement.getX(), setpoint.getX());
        double yOutput = yController.calculate(measurement.getY(), setpoint.getY());
        double thetaOutput = thetaController.calculate(measurement.getRotation().getRadians(), setpoint.getRotation().getRadians());

        return ChassisSpeeds.fromFieldRelativeSpeeds(
            xOutput,
            yOutput,
            thetaOutput,
            measurement.getRotation());
    }

    public ChassisSpeeds getError() {
        return new ChassisSpeeds(
            xController.getError(),
            yController.getError(),
            thetaController.getVelocityError());
    }
}
