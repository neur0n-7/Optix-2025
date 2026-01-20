package frc.robot.subsystems.djarm;

import edu.wpi.first.math.util.Units;

public class DJArmConstants {
    

    // Shoulder joint
    public static final double shoulderArmLengthMeters = Units.inchesToMeters(24);
    public static final double shoulderArmMassKg = Units.lbsToKilograms(7);
    public static final double shoulderReduction = 120.0; // motor rotations per joint rotation
    public static final double shoulderArmCOMMeters = shoulderArmLengthMeters / 2.0;
    public static final double shoulderkP = 30.0;
    public static final double shoulderkI = 0.0;
    public static final double shoulderkD = 0.0;
    public static final double shoulderkG = 0.0;
    public static final double shoulderkS = 0.0;
    public static final double shoulderkV = 0.0;
    public static final double shoulderkA = 0.0;

    // Elbow joint
    public static final double elbowArmLengthMeters = Units.inchesToMeters(18);
    public static final double elbowArmMassKg = Units.lbsToKilograms(5);
    public static final double elbowReduction = 75.0; // motor rotations per joint rotation
    public static final double elbowArmCOMMeters = elbowArmLengthMeters / 2.0;
    public static final double elbowkP = 30.0;
    public static final double elbowkI = 0.0;
    public static final double elbowkD = 0.0;
    public static final double elbowkG = 0.0;
    public static final double elbowkS = 0.0;
    public static final double elbowkV = 0.0;
    public static final double elbowkA = 0.0;

    // End effector
    public static final double endEffectorMassKg = Units.lbsToKilograms(1.5);
    public static final double endEffectorFingerLengthMeters = Units.inchesToMeters(5);
    
    public enum EndEffectorState {
        OPEN(false),
        CLOSED(true);

        public final boolean isClosed;

        EndEffectorState(boolean isClosed){
            this.isClosed = isClosed;
        }
    }

    public static final double shoulderDistalMassKg = shoulderArmMassKg + elbowArmMassKg + endEffectorMassKg;
    public static final double elbowDistalMassKg = elbowArmMassKg + endEffectorMassKg;

    public static final double PIDTolerance = Units.degreesToRadians(1.0);

    // Positions
    public enum DJArmStoredPoses {
        STOW(DJArmKinematics.calculate(0, shoulderArmLengthMeters - elbowArmLengthMeters)),
        HIGH(DJArmKinematics.calculate(0.6, 0.6)),
        FLOOR_INTAKE(DJArmKinematics.calculate(1.02, 0.05));

        public final DJArmPose pose;

        DJArmStoredPoses(DJArmPose pose){
            this.pose = pose;
        }
        
    }


}
