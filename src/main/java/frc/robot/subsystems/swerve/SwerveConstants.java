package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
// import frc.robot.utils.LoggedTunableNumber;

// TODO: use logged tunable numbers

public class SwerveConstants {

    public static final class ControlConstants {
        public static final double[] turnPID = new double[] { 3, 0, 0 };
        public static final double[] drivePID = new double[] { 0.27, 0, 0 };

        // our FF values
        public static double kSDriving = 0.26;
        public static double kVDriving = 2.765;
        public static double kADriving = 0.0;

        public static double maxVelocity = 4.3;
        public static double maxAcceleration = 3.3;

        // teleop speed
        public static final double teleopMaxSpeedReduction = 0.15; // Slow down by 15%
        public static final double teleopMaxSpeedMetersPerSecond = maxVelocity * (1 - teleopMaxSpeedReduction);

        public static final double maxAngularSpeedRadiansPerSecond = 11;
        public static final double maxAngularAccelerationRadiansPerSecondSquared = 9.0;

        // teleop angluar speed
        public static final double teleopMaxAngularSpeedReduction = 0.0;
        public static final double teleopMaxAngularSpeedRadPerSecond = maxAngularSpeedRadiansPerSecond
                * (1 - teleopMaxAngularSpeedReduction);
    }

    public static final class MotorConstants {
        public static final double driveMotorGearRatio = 6.75;
        public static final double turnMotorGearRatio = 12.8;

        // Module Settings: order is FL, FR, BL, BR
        public static final int[] driveMotorIds = { 3, 5, 7, 9 };
        public static final int[] turnMotorIds = { 4, 6, 8, 10 };
        public static final int[] absoluteEncoderIds = { 11, 12, 13, 14 };

        public static final boolean[] driveMotorInverted = {
                true,
                false,
                false,
                true
        };
        public static final boolean[] turningMotorInverted = {
                false,
                false,
                false,
                false
        };
        public static final boolean[] driveAbsoluteEncoderInverted = {
                false,
                false,
                false,
                false
        };
        public static final double[] absoluteEncoderOffsetRad = {
                0.417,
                4.380,
                4.791,
                0.845
        };
    }

    public static final class DrivetrainConstants {
        public static final double wheelDiameterMeters = Units.inchesToMeters(4);
        // Distance between right and left wheels

        public static final double trackWidth = Units.inchesToMeters(26.5);

        // dist between front and back wheels
        public static final double wheelBase = Units.inchesToMeters(20.5);

        public static final SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2, trackWidth / 2), // front left
                new Translation2d(wheelBase / 2, -trackWidth / 2), // front right
                new Translation2d(-wheelBase / 2, trackWidth / 2), // back left
                new Translation2d(-wheelBase / 2, -trackWidth / 2)); // back right

    }

}