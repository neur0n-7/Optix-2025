package frc.robot.subsystems.djarm.endeffector;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.subsystems.djarm.DJArmConstants.EndEffectorState;

public class RealEndEffector implements EndEffectorIO {

    private final Solenoid gripperSolenoid;
    private EndEffectorState gripperState = EndEffectorState.OPEN;

    public RealEndEffector(PneumaticsModuleType moduleType, int channel) {
        gripperSolenoid = new Solenoid(moduleType, channel);
    }

    @Override
    public void setState(EndEffectorState targetGripperState){
        gripperState = targetGripperState;
        gripperSolenoid.set(gripperState.isClosed); // closed = true, on
    }

    @Override
    public EndEffectorState getState() {
        return gripperState;
    }
}
