package frc.robot.subsystems.djarm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.djarm.joint.JointIO;

public class DJArmSubsystem extends SubsystemBase{
    
    private final JointIO shoulderJoint;
    private final JointIO elbowJoint;
    
    private final PIDController shoulderPID;
    private final PIDController elbowPID;

    private DJArmPose targetPose;

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

        targetPose = DJArmKinematics.calculate(0, DJArmConstants.shoulderArmLengthMeters-DJArmConstants.elbowArmLengthMeters);
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

        SmartDashboard.putNumber("DJArm/Shoulder/Current Degrees", shoulderRads);
        SmartDashboard.putNumber("DJArm/Shoulder/Target Degrees", targetPose.shoulderAngleRad());

        SmartDashboard.putNumber("DJArm/Elbow/Current Degrees", elbowRads);
        SmartDashboard.putNumber("DJArm/Elbow/Target Degrees", targetPose.elbowAngleRad());
    }

    @Override
    public void simulationPeriodic() {
        shoulderJoint.updateSimulation(0.02);
        elbowJoint.updateSimulation(0.02);
    }
}
