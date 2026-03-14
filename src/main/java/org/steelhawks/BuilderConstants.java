package org.steelhawks;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import org.steelhawks.subsystems.intake.IntakeConstants;
import org.steelhawks.util.LoggedTunableNumber;

public class BuilderConstants {

     public static class TurretPIDConstants {
        public static final LoggedTunableNumber kP = new LoggedTunableNumber("Turret kP", 0.0);
        public static final LoggedTunableNumber kI = new LoggedTunableNumber("Turret kI", 0.0);
        public static final LoggedTunableNumber kD = new LoggedTunableNumber("Turret kD", 0.0);
        public static final LoggedTunableNumber kA = new LoggedTunableNumber("Turret kA", 0.0);
        public static final LoggedTunableNumber kV = new LoggedTunableNumber("Turret kV", 0.0);
        public static final LoggedTunableNumber kS = new LoggedTunableNumber("Turret kS", 0.0);

        public static void setPID(TalonFX motor) {
            var rollerSlot0Configs = new Slot0Configs();
            rollerSlot0Configs
                    .withKA(kA.getAsDouble())
                    .withKP(kP.getAsDouble())
                    .withKI(kI.getAsDouble())
                    .withKD(kD.getAsDouble())
                    .withKS(kS.getAsDouble())
                    .withKV(kV.getAsDouble());

            motor.getConfigurator().apply(rollerSlot0Configs);
        }
    }

    public static class HoodPIDConstants {
        public static final LoggedTunableNumber kP = new LoggedTunableNumber("Hood kP", 0.0);
        public static final LoggedTunableNumber kI = new LoggedTunableNumber("Hood kI", 0.0);
        public static final LoggedTunableNumber kD = new LoggedTunableNumber("Hood kD", 0.0);
        public static final LoggedTunableNumber kA = new LoggedTunableNumber("Hood kA", 0.0);
        public static final LoggedTunableNumber kV = new LoggedTunableNumber("Hood kV", 0.0);
        public static final LoggedTunableNumber kS = new LoggedTunableNumber("Hood kS", 0.0);
        public static final LoggedTunableNumber kG = new LoggedTunableNumber("Hood kG", 0.0);

        public static void setPID(TalonFX motor) {
            var rollerSlot0Configs = new Slot0Configs();
            rollerSlot0Configs
                    .withKA(kA.getAsDouble())
                    .withKP(kP.getAsDouble())
                    .withKI(kI.getAsDouble())
                    .withKD(kD.getAsDouble())
                    .withKS(kS.getAsDouble())
                    .withKV(kV.getAsDouble())
                    .withKG(kG.getAsDouble());

            motor.getConfigurator().apply(rollerSlot0Configs);
        }
    }

    public static class FlywheelPIDConstants {
        public static final LoggedTunableNumber kP = new LoggedTunableNumber("Flywheel kP", 0.0);
        public static final LoggedTunableNumber kI = new LoggedTunableNumber("Flywheel kI", 0.0);
        public static final LoggedTunableNumber kD = new LoggedTunableNumber("Flywheel kD", 0.0);
        public static final LoggedTunableNumber kA = new LoggedTunableNumber("Flywheel kA", 0.0);
        public static final LoggedTunableNumber kV = new LoggedTunableNumber("Flywheel kV", 0.0);
        public static final LoggedTunableNumber kS = new LoggedTunableNumber("Flywheel kS", 0.0);

        public static void setPID(TalonFX motor) {
            var rollerSlot0Configs = new Slot0Configs();
            rollerSlot0Configs
                    .withKA(kA.getAsDouble())
                    .withKP(kP.getAsDouble())
                    .withKI(kI.getAsDouble())
                    .withKD(kD.getAsDouble())
                    .withKS(kS.getAsDouble())
                    .withKV(kV.getAsDouble());

            motor.getConfigurator().apply(rollerSlot0Configs);
        }
    }

    public static class ExtensionPIDConstants {
        public static final LoggedTunableNumber kP = new LoggedTunableNumber("Extension kP", 0.0);
        public static final LoggedTunableNumber kI = new LoggedTunableNumber("Extension kI", 0.0);
        public static final LoggedTunableNumber kD = new LoggedTunableNumber("Extension kD", 0.0);
        public static final LoggedTunableNumber kA = new LoggedTunableNumber("Extension kA", 0.0);
        public static final LoggedTunableNumber kV = new LoggedTunableNumber("Extension kV", 0.0);
        public static final LoggedTunableNumber kS = new LoggedTunableNumber("Extension kS", 0.0);
        public static final LoggedTunableNumber kG = new LoggedTunableNumber("Extension kG", 0.0);

        public static void setPID(TalonFX motor) {
            var rollerSlot0Configs = new Slot0Configs();
            rollerSlot0Configs
                    .withKA(kA.getAsDouble())
                    .withKP(kP.getAsDouble())
                    .withKI(kI.getAsDouble())
                    .withKD(kD.getAsDouble())
                    .withKS(kS.getAsDouble())
                    .withKV(kV.getAsDouble())
                    .withKG(kG.getAsDouble());

            motor.getConfigurator().apply(rollerSlot0Configs);
        }
    }

    public static class RollerPIDConstants {
        public static final LoggedTunableNumber kP = new LoggedTunableNumber("Roller kP", 0.0);
        public static final LoggedTunableNumber kI = new LoggedTunableNumber("Roller kI", 0.0);
        public static final LoggedTunableNumber kD = new LoggedTunableNumber("Roller kD", 0.0);
        public static final LoggedTunableNumber kA = new LoggedTunableNumber("Roller kA", 0.0);
        public static final LoggedTunableNumber kV = new LoggedTunableNumber("Roller kV", 0.0);
        public static final LoggedTunableNumber kS = new LoggedTunableNumber("Roller kS", 0.0);

        public static void setPID(TalonFX motor) {
            var rollerSlot0Configs = new Slot0Configs();
            rollerSlot0Configs
                    .withKA(kA.getAsDouble())
                    .withKP(kP.getAsDouble())
                    .withKI(kI.getAsDouble())
                    .withKD(kD.getAsDouble())
                    .withKS(kS.getAsDouble())
                    .withKV(kV.getAsDouble());

            motor.getConfigurator().apply(rollerSlot0Configs);
        }
    }


}
