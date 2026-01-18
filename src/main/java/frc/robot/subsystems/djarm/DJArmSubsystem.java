package frc.robot.subsystems.djarm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.djarm.joint.JointIO;

public class DJArmSubsystem extends SubsystemBase{
    
    private final JointIO shoulderJoint;
    private final JointIO elbowJoint;
    
    private final PIDController shoulderPID;
    private final PIDController elbowPID;

    private DJArmPose targetPose;

    private final Mechanism2d mech2d;
    private final MechanismLigament2d shoulderLigament;
    private final MechanismLigament2d elbowLigament;

    public DJArmSubsystem(JointIO shoulderJoint, JointIO elbowJoint){
        this.shoulderJoint = shoulderJoint;
        this.elbowJoint = elbowJoint;

        shoulderPID = new PIDController(
            DJArmConstants.shoulderkP,
            DJArmConstants.shoulderkI,
            DJArmConstants.shoulderkD
        );

        elbowPID = new PIDController(
            DJArmConstants.elbowkP,
            DJArmConstants.elbowkI,
            DJArmConstants.elbowkD
        );

        shoulderPID.setTolerance(DJArmConstants.PIDTolerance);
        elbowPID.setTolerance(DJArmConstants.PIDTolerance);

        targetPose = DJArmKinematics.calculate(0, DJArmConstants.shoulderArmLengthMeters-DJArmConstants.elbowArmLengthMeters);

        mech2d = new Mechanism2d(3.0, 3.0);

        MechanismRoot2d root =
            mech2d.getRoot("ShoulderRoot", 1.5, 0.5);

        shoulderLigament =
            root.append(
                new MechanismLigament2d(
                    "Shoulder",
                    DJArmConstants.shoulderArmLengthMeters,
                    90.0
                )
            );

        elbowLigament =
            shoulderLigament.append(
                new MechanismLigament2d(
                    "Elbow",
                    DJArmConstants.elbowArmLengthMeters,
                    0.0
                )
            );
    }

    public void setTargetPose(DJArmPose pose){
        targetPose = pose;
        shoulderPID.setSetpoint(targetPose.shoulderAngleRad());
        elbowPID.setSetpoint(targetPose.elbowAngleRad());
    }

    public boolean atTarget(){
        return shoulderPID.atSetpoint() && elbowPID.atSetpoint();
    }

    @Override
    public void periodic() {
        double shoulderRads = shoulderJoint.getPositionRads();
        shoulderJoint.setVoltage(shoulderPID.calculate(shoulderRads));
        double elbowRads = elbowJoint.getPositionRads();
        elbowJoint.setVoltage(elbowPID.calculate(elbowRads));

        SmartDashboard.putBoolean("DJArm/Shoulder/AtSetpoint", shoulderPID.atSetpoint());
        SmartDashboard.putNumber("DJArm/Shoulder/Current Radians", shoulderRads);
        SmartDashboard.putNumber("DJArm/Shoulder/Target Radians", targetPose.shoulderAngleRad());
        
        SmartDashboard.putBoolean("DJArm/Elbow/AtSetpoint", elbowPID.atSetpoint());
        SmartDashboard.putNumber("DJArm/Elbow/Current Radians", elbowRads);
        SmartDashboard.putNumber("DJArm/Elbow/Target Radians", targetPose.elbowAngleRad());

        SmartDashboard.putData("DJArm/mech", mech2d);

        shoulderLigament.setAngle(Units.radiansToDegrees(shoulderRads));
        elbowLigament.setAngle(Units.radiansToDegrees(elbowRads));
    }

    @Override
    public void simulationPeriodic() {
        shoulderJoint.updateSimulation(0.02);
        elbowJoint.updateSimulation(0.02);
    }
}
