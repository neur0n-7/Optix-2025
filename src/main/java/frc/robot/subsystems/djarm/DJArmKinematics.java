package frc.robot.subsystems.djarm;

public class DJArmKinematics {
    public static DJArmPose calculate(double xTarget, double yTarget) {

        double L1 = DJArmConstants.shoulderArmLengthMeters;
        double L2 = DJArmConstants.elbowArmLengthMeters;

        double c2 = (xTarget*xTarget + yTarget*yTarget - L1*L1 - L2*L2) / (2*L1*L2);
        double s2 = Math.sqrt(1 - c2*c2); // elbow-down solution
        double theta2 = Math.atan2(s2, c2);

        double k1 = L1 + L2 * Math.cos(theta2);
        double k2 = L2 * Math.sin(theta2);
        double theta1 = Math.atan2(yTarget, xTarget) - Math.atan2(k2, k1);

        return new DJArmPose(theta1, theta2);
    }
}
