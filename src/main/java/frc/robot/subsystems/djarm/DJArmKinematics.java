package frc.robot.subsystems.djarm;

import edu.wpi.first.math.MathUtil;

public class DJArmKinematics {


    public static boolean isValid(double xTarget, double yTarget) {

        double L1 = DJArmConstants.shoulderArmLengthMeters;
        double L2 = DJArmConstants.elbowArmLengthMeters;
        double r = Math.hypot(xTarget, yTarget);

        // Reachability check
        if (r > L1 + L2 || r < Math.abs(L1 - L2)) {
            return false;
        } else {
            return true;
        }

    }

    public static DJArmPose calculate(double xTarget, double yTarget) {

        double L1 = DJArmConstants.shoulderArmLengthMeters;
        double L2 = DJArmConstants.elbowArmLengthMeters;

        double c2 = (xTarget*xTarget + yTarget*yTarget - L1*L1 - L2*L2) / (2*L1*L2);
        c2 = MathUtil.clamp(c2, -1.0, 1.0);

        double s2 = -Math.sqrt(1 - c2*c2); // elbow-up solution
        double theta2 = Math.atan2(s2, c2);

        double k1 = L1 + L2 * Math.cos(theta2);
        double k2 = L2 * Math.sin(theta2);
        double theta1 = Math.atan2(yTarget, xTarget) - Math.atan2(k2, k1);

        return new DJArmPose(theta1, theta2, xTarget, yTarget);
    }
}
