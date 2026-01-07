package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.arm.ArmConstants.ArmPositionStates;
import frc.robot.subsystems.arm.ArmConstants.CargoStates;
import frc.robot.subsystems.arm.ArmConstants.GripperStates;
import frc.robot.subsystems.arm.gripper.GripperIO;
import frc.robot.subsystems.arm.motor.ArmMotorIO;

/**
 * Single-jointed arm subsystem class
 * @author Anish Gupta
 */
public class ArmSubsystem extends SubsystemBase {

    private final ArmMotorIO armMotor;
    private final GripperIO gripper;

    private final ProfiledPIDController pidController;
    private final ArmFeedforward emptyFeedforward;
    private final ArmFeedforward loadedFeedforward;
    
    private double lastSetpointVelocity = 0.0;
    private double lastActualVelocityRads = 0.0;
    private double targetPositionDegrees = 0.0;
    
    private ArmPositionStates currentPositionState = ArmPositionStates.STOW;

    private final LoggedMechanism2d mech = new LoggedMechanism2d(4, 4);
    private final LoggedMechanismLigament2d armMech;
    private final LoggedMechanismLigament2d gripperLeft;
    private final LoggedMechanismLigament2d gripperRight;
    
    public ArmSubsystem(ArmMotorIO motor, GripperIO gripper) {
        this.armMotor = motor;
        this.gripper = gripper;

        pidController = new ProfiledPIDController(
                ArmConstants.kP,
                ArmConstants.kI,
                ArmConstants.kD,
                new TrapezoidProfile.Constraints(
                    ArmConstants.maxEmptyVelocityRads,
                    ArmConstants.maxEmptyAccelRads
                )
        );

        lastSetpointVelocity = pidController.getSetpoint().velocity;


        pidController.setTolerance(Units.degreesToRadians(ArmConstants.PIDToleranceDegrees));

        // FF for arm when not holding cone
        emptyFeedforward = new ArmFeedforward(
                ArmConstants.kS,
                ArmConstants.kGEmpty,
                ArmConstants.kVEmpty,
                ArmConstants.kAEmpty
        );

        // FF for arm when holding cone
        loadedFeedforward = new ArmFeedforward(
            ArmConstants.kS,
            ArmConstants.kGLoaded,
            ArmConstants.kVLoaded,
            ArmConstants.kALoaded
        );

        LoggedMechanismRoot2d root = mech.getRoot("arm", 2, 2);

        Color8Bit orange = new Color8Bit(255, 165, 0);

        armMech = root.append(
                new LoggedMechanismLigament2d(
                        "arm",
                        1.5,
                        0,
                        12,
                        orange
                )
        );

        gripperLeft = armMech.append(
            new LoggedMechanismLigament2d(
                "gripperLeft",
                0.3,
                0,
                12,
                orange
            )
        );

        gripperRight = armMech.append(
            new LoggedMechanismLigament2d(
                "gripperRight",
                0.3,
                0,
                12,
                orange
            )
        ); 
    }

    /**
     * Set the target POSITION state of the arm
     * e.g setTargetPositionState(ArmPositionStates.STOW) will set the target position to the arm to STOW.
     */
    public void setTargetPositionState(ArmPositionStates targetPositionState) {
        pidController.setGoal(Units.degreesToRadians(targetPositionState.position_degs));
        targetPositionDegrees = targetPositionState.position_degs;
        currentPositionState = targetPositionState;
    }

    /**
     * Set the state of the grippper to open or closed.
     * @param gripperState Target gripper state
     */
    public void setGripperState(GripperStates gripperState){
        gripper.setGripperState(gripperState);
    }

    /**
     * Sets the cargo state (whether or not the robot is holding a cone) to the target state.
     * @param cargoState Target CargoStates value- EMPTY or LOADED
     */
    public void setCargoState(CargoStates cargoState){
        gripper.setCargoState(cargoState);
        armMotor.setSimArmMass(cargoState.isHoldingCone);
    }

    /**
     * Get whether or not the arm is at its set target position.
     * @return boolean representing whether or not at the target position
     */
    public boolean atPositionTarget() {
        return pidController.atGoal();
    }

    /**
     * Get the current position (angle) of the arm
     * @return A double rpresenting the position of the arm in degrees.
     */
    public double getPositionDegrees() {
        return armMotor.getPositionDegrees();
    }

    /**
     * Get the target position state of the arm.
     * @return An ArmPositionStates object representing the state.
     */
    public ArmPositionStates getPositionState() {
        return currentPositionState;
    }

    /**
     * Get the current gripper state of the arm.
     * @return A GripperStates object representing the state.
     */
    public GripperStates getGripperState() {
        return gripper.getGripperState();
    }

    /**
     * Get the current cargo state of the arm (whether or not a cone is loaded)
     * @return A CargoStates object representing the state.
     */
    public CargoStates getCargoState() {
        return gripper.getCargoState();
    }

    /**
     * Send the passed number of volts to the arm's motor
     * @param volts A double representing the number of volts
     */
    public void setMotorVoltage(double volts){
        armMotor.setVoltage(MathUtil.clamp(volts, -12, 12));
    }

    @Override
    public void periodic() {
                
        TrapezoidProfile.State setpoint = pidController.getSetpoint();

        // Setpoint velocity/acceleration
        double setpointVelocity = setpoint.velocity;
        double setpointAcceleration = (setpointVelocity - lastSetpointVelocity) / 0.02;
        lastSetpointVelocity = setpointVelocity;

        // Actual velocity/acceleration
        boolean holdingCone = gripper.getCargoState().isHoldingCone;
        double maxAccel = holdingCone ? ArmConstants.maxLoadedAccelRads : ArmConstants.maxEmptyAccelRads;
        double maxVel   = holdingCone ? ArmConstants.maxLoadedVelocityRads : ArmConstants.maxEmptyVelocityRads;
        double actualVelocityRads = MathUtil.clamp(armMotor.getVelocityRadPerSec(), -maxVel, maxVel);
        double actualAccelerationRads = MathUtil.clamp((actualVelocityRads - lastActualVelocityRads) / 0.02, -maxAccel, maxAccel);
        lastActualVelocityRads = actualVelocityRads;

        // PID
        double currentDegrees = getPositionDegrees();
        double currentRadians = Units.degreesToRadians(currentDegrees);
        double pidVolts = MathUtil.clamp(pidController.calculate(currentRadians), -12, 12);

        // FF
        double ffVolts;
        if (holdingCone){
            ffVolts = loadedFeedforward.calculate(Units.degreesToRadians(getPositionDegrees() - 90), setpointVelocity, setpointAcceleration);
        } else {
            ffVolts = emptyFeedforward.calculate(Units.degreesToRadians(getPositionDegrees() - 90), setpointVelocity, setpointAcceleration);
        }
        ffVolts = MathUtil.clamp(ffVolts, -12, 12);


        // Send total voltage to arm motor
        double totalVolts = MathUtil.clamp(pidVolts + ffVolts, -12, 12);
        setMotorVoltage(totalVolts);

        // Logging
        SmartDashboard.putNumber("Arm/Current Degrees", currentDegrees);
        SmartDashboard.putNumber("Arm/Target Degrees", targetPositionDegrees);

        SmartDashboard.putNumber("Arm/Volts PID", pidVolts);
        SmartDashboard.putNumber("Arm/Volts FF", ffVolts);
        SmartDashboard.putNumber("Arm/Volts TOTAL", totalVolts);
        SmartDashboard.putBoolean("Arm/At Target", atPositionTarget());

        SmartDashboard.putNumber("Arm/Velocity (Setpoint)", setpointVelocity);
        SmartDashboard.putNumber("Arm/Velocity (Actual)", actualVelocityRads);
        SmartDashboard.putNumber("Arm/Acceleration (Setpoint)", setpointAcceleration);
        SmartDashboard.putNumber("Arm/Acceleration (Actual)", actualAccelerationRads);

        SmartDashboard.putData("Arm/Mech2d", mech);

        SmartDashboard.putString("Arm/Position State", getPositionState().toString());
        SmartDashboard.putString("Arm/Gripper State", getGripperState().toString());
        SmartDashboard.putString("Arm/Cargo State", getCargoState().toString());
    }

    // Update the LoggedMechanism2d
    public void updateMechanism(){
        double armAngleDegrees = getPositionDegrees();

        // Currently angles are stored with 0 degrees pointing down.
        // However, 0 degrees in Mechanism2ds points right
        armMech.setAngle(armAngleDegrees - 90);

        double clawAngle = gripper.getGripperState() == GripperStates.OPEN ? 60.0 : 10.0;
        gripperLeft.setAngle(-clawAngle);
        gripperRight.setAngle(clawAngle);

    }

    @Override
    public void simulationPeriodic(){
        armMotor.updateSimulation(0.02);
        updateMechanism();
    }
}
