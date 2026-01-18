package frc.robot.subsystems.djarm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.djarm.DJArmConstants.DJArmStoredPoses;
import frc.robot.subsystems.djarm.joint.JointIO;

public class DJArmSubsystem extends SubsystemBase{
    
    private final JointIO shoulderJoint;
    private final JointIO elbowJoint;
    
    private final PIDController shoulderPID;
    private final PIDController elbowPID;

    private DJArmStoredPoses targetPose;

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

        targetPose = DJArmConstants.DJArmStoredPoses.STOW;

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

    public void setTargetPose(DJArmStoredPoses pose){
        targetPose = pose;
        shoulderPID.setSetpoint(targetPose.pose.shoulderAngleRad());
        elbowPID.setSetpoint(targetPose.pose.elbowAngleRad());
    }

    public boolean atTarget(){
        return shoulderPID.atSetpoint() && elbowPID.atSetpoint();
    }

    @Override
    public void periodic() {
        double shoulderRads = shoulderJoint.getPositionRads();
        double shoulderPidVolts = shoulderPID.calculate(shoulderRads);
        shoulderJoint.setVoltage(shoulderPidVolts);

        double elbowRads = elbowJoint.getPositionRads();
        double elbowPidVolts = elbowPID.calculate(elbowRads);
        elbowJoint.setVoltage(elbowPidVolts);

        SmartDashboard.putBoolean("DJArm/Shoulder/AtSetpoint", shoulderPID.atSetpoint());
        SmartDashboard.putNumber("DJArm/Shoulder/Current Radians", shoulderRads);
        SmartDashboard.putNumber("DJArm/Shoulder/Target Radians", targetPose.pose.shoulderAngleRad());
        SmartDashboard.putNumber("DJArm/Shoulder/Velocity", shoulderJoint.getVelocityRadPerSec());        
        SmartDashboard.putNumber("DJArm/Shoulder/Volts PID", shoulderPidVolts);        
        
        
        SmartDashboard.putBoolean("DJArm/Elbow/AtSetpoint", elbowPID.atSetpoint());
        SmartDashboard.putNumber("DJArm/Elbow/Current Radians", elbowRads);
        SmartDashboard.putNumber("DJArm/Elbow/Target Radians", targetPose.pose.elbowAngleRad());
        SmartDashboard.putNumber("DJArm/Elbow/Velocity", elbowJoint.getVelocityRadPerSec());   
        SmartDashboard.putNumber("DJArm/Elbow/Volts PID", elbowPidVolts);        
     


        SmartDashboard.putString("DJArm/Target Pose", targetPose.toString());
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
