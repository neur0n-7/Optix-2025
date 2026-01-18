package frc.robot.subsystems.djarm.joint;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class SimArmJoint implements JointIO {

    private SingleJointedArmSim sim;
    private double voltage = 0.0;

    public SimArmJoint(
        double gearing,
        double jointMassKg,
        double jointLengthMeters
    ) {

        double momentOfInertia = (1.0/3.0) * jointMassKg * Math.pow(jointLengthMeters, 2);

        sim = new SingleJointedArmSim(
                DCMotor.getNEO(1),
                gearing,
                momentOfInertia,
                jointLengthMeters,
                Units.degreesToRadians(-360.0),
                Units.degreesToRadians(360.0),
                false,
                Units.degreesToRadians(0.0)
        );
    }

    @Override
    public void setVoltage(double volts) {
        voltage = MathUtil.clamp(volts, -12.0, 12.0);
        sim.setInputVoltage(voltage);
    }

    @Override
    public double getVelocityRadPerSec() {
        return sim.getVelocityRadPerSec();
    }

    @Override
    public double getPositionRads() {
        return sim.getAngleRads();
    }

    public void updateSimulation(double dtSeconds){
        sim.update(dtSeconds);
    }  

}