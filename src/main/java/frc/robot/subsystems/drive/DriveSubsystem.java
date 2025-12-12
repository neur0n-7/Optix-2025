package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.NeoMotor;

public class DriveSubsystem extends SubsystemBase {

	private final NeoMotor motor; // new SparkMax(22, MotorType.kBrushless);
	private final PIDController pid = new PIDController(
			DriveConstants.kP,
			DriveConstants.kI,
			DriveConstants.kD);

	private double targetDegrees = 0.0;

	public DriveSubsystem(NeoMotor motor) {
		pid.setTolerance(DriveConstants.PIDTolerance);
		this.motor = motor;

	}

	public void setTargetDegrees(double degrees) {
		targetDegrees = degrees;
		pid.reset();
	}


	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		double currentVoltage = motor.getAppliedOutput() * motor.getBusVoltage();
		double currentDegrees = motor.getPosition();

		double output = pid.calculate(currentDegrees, targetDegrees);

		output = MathUtil.clamp(output, -12.0, 12.0);

		motor.setVoltage(output);

		// logging
		SmartDashboard.putNumber("Drive/TargetDegs", targetDegrees);
		SmartDashboard.putNumber("Drive/ActualVoltage", currentVoltage);
		SmartDashboard.putNumber("Drive/PIDOutputVoltage", output);

	}

	@Override
	public void simulationPeriodic() { }
}
