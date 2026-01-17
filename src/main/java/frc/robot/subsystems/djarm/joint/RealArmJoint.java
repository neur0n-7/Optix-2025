package frc.robot.subsystems.djarm.joint;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class RealArmJoint implements JointIO {

    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private final double gearing;
    
    public RealArmJoint(
        int canId,
        double gearing
    ) {

        motor = new SparkMax(canId, MotorType.kBrushless);
        encoder = motor.getEncoder();
        encoder.setPosition(0.0);

        this.gearing = gearing;

    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(MathUtil.clamp(volts, -12.0, 12.0));
    }

    @Override
    public double getVelocityRadPerSec() {
        double motorRpm = encoder.getVelocity();
        double armDegreesPerMin = motorRpm * 360.0 / gearing;
        return Units.degreesToRadians(armDegreesPerMin / 60.0) ;
    }

    @Override
    public double getPositionRads() {
        return Math.toDegrees(encoder.getPosition() * 360.0 / gearing);
    }

}