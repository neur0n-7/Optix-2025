package frc.robot.subsystems.akit_testing;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.LoggedTunableNumber;

public class Flywheel extends SubsystemBase{
    private static LoggedTunableNumber kPLogger = new LoggedTunableNumber("Flywheel/kP", 0.0);

    private double targetVelcoity = 0.0;
    private double velocity = 0.0;

    @Override
    public void periodic(){
        double currentKP = kPLogger.get();

        double error  = targetVelcoity - velocity;
        double output = currentKP * error;
        
        velocity += output * 0.02;

        Logger.recordOutput("Flywheel/Velocity", velocity);
        Logger.recordOutput("Flywheel/Error", error);
        


    }
}
