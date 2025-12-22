package frc.robot.subsystems.elevatorV2;

import edu.wpi.first.math.util.Units;

public class V2ElevatorConstants {

    // PID
    public static final double kP = 5;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    // FF
    public static final double kS = 0.16; // friction
    public static final double kG = 0.32; // gravity
    public static final double kV = 1;  // velocity
    public static final double kA = 0.1; // accel

    // contraints
    public static final double maxVelocity = 2.0;  // m/sec
    public static final double maxAccel = 3.0;     // m/sec^2

    // specs
    public static final double elevatorBaseHeight = Units.feetToMeters(3.25);
    public static final double drumDiameter = Units.inchesToMeters(1.5);
    public static final double gearing = 16 * (24.0 / 22.0); 
    public static final double drumCircumference = Math.PI * drumDiameter;
    public static final double metersPerMotorRotation = drumCircumference / gearing;
    public static final double carriageMassKg = 5.0;

    public enum ElevatorStates {
        LOWEST(0.0),
        MIDDLE(Units.feetToMeters(3)),
        HIGHEST(Units.feetToMeters(6));

        public final double position;

        ElevatorStates(double position) {
            this.position = position;
        }
    }
}
