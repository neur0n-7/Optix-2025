package frc.robot.subsystems.djarm;

import edu.wpi.first.math.util.Units;

public class DJArmConstants {
    

    // Shoulder joint
    public static final double shoulderArmLengthMeters = Units.inchesToMeters(24);
    public static final double shoulderArmMassKg = Units.lbsToKilograms(7);
    public static final double shoulderReduction = 120.0; // motor rotations per joint rotation
    public static final double shoulderArmCOMMeters = shoulderArmLengthMeters / 2.0;
    public static final double shoulderkP = 0.0;
    public static final double shoulderkI = 0.0;
    public static final double shoulderkD = 0.0;
    public static final double shoulderkG = 0.0;
    public static final double shoulderkS = 0.0;
    public static final double shoulderkV = 0.0;
    public static final double shoulderkA = 0.0;

    // Elbow joint
    public static final double elbowLengthMeters = Units.inchesToMeters(18);
    public static final double elbowArmMassKg = Units.lbsToKilograms(5);
    public static final double elbowReduction = 75.0; // motor rotations per joint rotation
    public static final double elbowArmCOMMeters = elbowLengthMeters / 2.0;
    public static final double elbowkP = 0.0;
    public static final double elbowkI = 0.0;
    public static final double elbowkD = 0.0;
    public static final double elbowkG = 0.0;
    public static final double elbowkS = 0.0;
    public static final double elbowkV = 0.0;
    public static final double elbowkA = 0.0;

    public static final double shoulderDistalMassKg = shoulderArmMassKg + elbowArmMassKg;
    public static final double elbowDistalMassKg = elbowArmMassKg;


}
