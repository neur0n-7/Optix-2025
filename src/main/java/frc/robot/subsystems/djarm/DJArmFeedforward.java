package frc.robot.subsystems.djarm;

public class DJArmFeedforward {

    private static final double g = 9.80665;

    public static double calculateShoulder(
            double shoulderAngleRad,
            double shoulderVelocityRadPerSec,
            double shoulderAccelRadPerSec2) {

        double shoulderArmTorque = DJArmConstants.shoulderArmMassKg
                * g
                * DJArmConstants.shoulderArmCOMMeters
                * Math.cos(shoulderAngleRad);

        double elbowArmTorque = DJArmConstants.elbowArmMassKg
                * g
                * (DJArmConstants.shoulderArmLengthMeters
                        + DJArmConstants.elbowArmLengthMeters / 2.0)
                * Math.cos(shoulderAngleRad);

        double totalTorqueNm = shoulderArmTorque + elbowArmTorque;
        double kGVolts = torqueToVolts(totalTorqueNm, DJArmConstants.shoulderReduction);

        return kGVolts
                + DJArmConstants.shoulderkS * Math.signum(shoulderVelocityRadPerSec)
                + DJArmConstants.shoulderkV * shoulderVelocityRadPerSec
                + DJArmConstants.shoulderkA * shoulderAccelRadPerSec2;
    }

    public static double calculateElbow(
            double shoulderAngleRad,
            double elbowAngleRad,
            double elbowVelocityRadPerSec,
            double elbowAccelRadPerSec2) {

        double absoluteElbowAngle = shoulderAngleRad + elbowAngleRad;

        double elbowArmTorque = DJArmConstants.elbowArmMassKg
                * g
                * (DJArmConstants.elbowArmLengthMeters / 2.0)
                * Math.cos(absoluteElbowAngle);

        double totalTorqueNm = elbowArmTorque;

        double kGVolts = torqueToVolts(totalTorqueNm, DJArmConstants.elbowReduction);

        return kGVolts
                + DJArmConstants.elbowkS * Math.signum(elbowVelocityRadPerSec)
                + DJArmConstants.elbowkV * elbowVelocityRadPerSec
                + DJArmConstants.elbowkA * elbowAccelRadPerSec2;
    }


    private static double torqueToVolts(double torqueNm, double reduction) {
        double motorTorqueConstant = 0.018;

        double motorTorque = torqueNm / reduction;
        double motorCurrent = motorTorque / motorTorqueConstant;

        return motorCurrent * (12.0 / 100.0);
    }
}
