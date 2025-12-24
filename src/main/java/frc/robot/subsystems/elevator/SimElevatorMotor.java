package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.subsystems.drive.NeoMotor;
import edu.wpi.first.math.MathUtil;

public class SimElevatorMotor implements NeoMotor {

    private final ElevatorSim sim;
    private double voltage = 0.0;

    public SimElevatorMotor() {
        sim = new ElevatorSim(
                DCMotor.getNEO(1),
                ElevatorConstants.gearing,
                ElevatorConstants.carriageMassKg,
                ElevatorConstants.drumDiameter / 2.0,
                0.0,
                ElevatorConstants.ElevatorStates.HIGHEST.position,
                true,
                0.0);
    }

    @Override
    public void setVoltage(double volts) {
        voltage = MathUtil.clamp(volts, -12.0, 12.0);
        sim.setInputVoltage(voltage);
    }

    @Override
    public double getBusVoltage() {
        return RoboRioSim.getVInVoltage();
    }

    @Override
    public double getAppliedOutput() {
        return voltage / 12.0;
    }

    @Override
    public double getVelocity() {
        return sim.getVelocityMetersPerSecond();
    }

    @Override
    public double getPositionMeters() {
        return sim.getPositionMeters();
    }

    @Override
    public void updateSimulation(double dtSeconds) {
        sim.update(dtSeconds);
    }
}
