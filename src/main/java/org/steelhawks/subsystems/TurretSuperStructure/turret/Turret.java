package org.steelhawks.subsystems.TurretSuperStructure.turret;

import com.ctre.phoenix6.controls.VelocityVoltage;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {

    private final TurretIO io;
    private final VelocityVoltage velocityVoltage
    private final double outputFF;

    private TurretIOInputsAutoLogged inputs;

    public Turret(TurretIO io) {
        this.io = io;

        TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(80, 160));

        TrapezoidProfile.State goal = new TrapezoidProfile.State(200, 0);
        TrapezoidProfile.State set_point = new TrapezoidProfile.State();

        set_point = profile.calculate(0.020, set_point, goal);

        outputFF = (TurretConstants.SHOOTER_KS.getAsDouble() * Math.signum(set_point.position))
                + (TurretConstants.SHOOTER_KV.getAsDouble() * set_point.position)
                + (TurretConstants.SHOOTER_KA.getAsDouble() * set_point.velocity);

    }

    @Override
    public void periodic() {
        io.turnTurret(inputs, 5.0, outputFF);
    }


}
