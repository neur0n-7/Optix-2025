package frc.robot.subsystems.djarm;

import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.djarm.DJArmConstants.DJArmStoredPoses;
import frc.robot.subsystems.djarm.DJArmConstants.EndEffectorState;
import frc.robot.subsystems.djarm.endeffector.EndEffectorIO;
import frc.robot.subsystems.djarm.joint.JointIO;

public class DJArmSubsystem extends SubsystemBase{
    
    private final JointIO shoulderJoint;
    private final JointIO elbowJoint;
    private final EndEffectorIO endEffector;
    
    private final PIDController shoulderPID;
    private final PIDController elbowPID;

    private DJArmPose targetPose;

    private final LoggedMechanism2d mech2d;
    private final LoggedMechanismLigament2d shoulderLigament;
    private final LoggedMechanismLigament2d elbowLigament;
    private final LoggedMechanismLigament2d endEffectorLeft;
    private final LoggedMechanismLigament2d endEffectorRight;

    public DJArmSubsystem(JointIO shoulderJoint, JointIO elbowJoint, EndEffectorIO endEffector){
        this.shoulderJoint = shoulderJoint;
        this.elbowJoint = elbowJoint;
        this.endEffector = endEffector;

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

        setTargetPose(DJArmStoredPoses.STOW.pose);
    
        mech2d = new LoggedMechanism2d(2.5, 2.0);

        LoggedMechanismRoot2d root = mech2d.getRoot("ShoulderRoot", 1.25, 0.75);

        shoulderLigament =
            root.append(
                new LoggedMechanismLigament2d(
                    "Shoulder",
                    DJArmConstants.shoulderArmLengthMeters,
                    90.0
                )
            );

        elbowLigament =
            shoulderLigament.append(
                new LoggedMechanismLigament2d(
                    "Elbow",
                    DJArmConstants.elbowArmLengthMeters,
                    0.0
                )
            );

        endEffectorLeft = elbowLigament.append(
            new LoggedMechanismLigament2d(
                "endEffectorLeft",
                DJArmConstants.endEffectorFingerLengthMeters,
                0)
            );

        endEffectorRight = elbowLigament.append(
            new LoggedMechanismLigament2d(
                "endEffectorRight",
                DJArmConstants.endEffectorFingerLengthMeters,
                0)
            );

        
    }

    public void setTargetPose(DJArmPose pose){
        if (DJArmKinematics.isValid(pose.xTarget(), pose.yTarget())){
            targetPose = pose;
            shoulderPID.setSetpoint(targetPose.shoulderAngleRad());
            elbowPID.setSetpoint(targetPose.elbowAngleRad());
        }
    }

    public void shiftTargetPose(double xShift, double yShift){
        double xNew = targetPose.xTarget() + xShift;
        double yNew = targetPose.yTarget() + yShift;
        setTargetPose(DJArmKinematics.calculate(xNew, yNew));
    }

    public void setEndEffectorState(EndEffectorState state){
        endEffector.setState(state);
    }

    public EndEffectorState getEndEffectorState(){
        return endEffector.getState();
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
        SmartDashboard.putNumber("DJArm/Shoulder/Target Radians", targetPose.shoulderAngleRad());
        SmartDashboard.putNumber("DJArm/Shoulder/Velocity", shoulderJoint.getVelocityRadPerSec());        
        SmartDashboard.putNumber("DJArm/Shoulder/Volts PID", shoulderPidVolts);        
        
        SmartDashboard.putBoolean("DJArm/Elbow/AtSetpoint", elbowPID.atSetpoint());
        SmartDashboard.putNumber("DJArm/Elbow/Current Radians", elbowRads);
        SmartDashboard.putNumber("DJArm/Elbow/Target Radians", targetPose.elbowAngleRad());
        SmartDashboard.putNumber("DJArm/Elbow/Velocity", elbowJoint.getVelocityRadPerSec());   
        SmartDashboard.putNumber("DJArm/Elbow/Volts PID", elbowPidVolts);       
        
        SmartDashboard.putNumber("DJArm/EndEffector/X", targetPose.xTarget());
        SmartDashboard.putNumber("DJArm/EndEffector/Y", targetPose.yTarget());
        SmartDashboard.putString("DJArm/EndEffector/State", endEffector.getState().toString());

        
     
        SmartDashboard.putData("DJArm/mech", mech2d);

        shoulderLigament.setAngle(Units.radiansToDegrees(shoulderRads));
        elbowLigament.setAngle(Units.radiansToDegrees(elbowRads));
    }

    @Override
    public void simulationPeriodic() {
        shoulderJoint.updateSimulation(0.02);
        elbowJoint.updateSimulation(0.02);

        double endEffectorAngle = endEffector.getState() == EndEffectorState.OPEN ? 60.0 : 10.0;
        endEffectorLeft.setAngle(-endEffectorAngle);
        endEffectorRight.setAngle(endEffectorAngle);
    }
}
