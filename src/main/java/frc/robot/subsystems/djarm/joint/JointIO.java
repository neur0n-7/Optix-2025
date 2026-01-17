package frc.robot.subsystems.djarm.joint;

public interface JointIO {

    void setVoltage(double volts);

    double getVelocityRadPerSec();

    double getPositionRads();

    default void updateSimulation(double dtSeconds) { }
}