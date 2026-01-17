package frc.robot.subsystems.djarm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.djarm.joint.JointIO;

public class DJArmSubsystem extends SubsystemBase{
    
    private final JointIO shoulderJoint;
    private final JointIO elbowJoint;
    
    private final PIDController shoulderPID;
    private final PIDController elbowPID;

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

        
    }
}
