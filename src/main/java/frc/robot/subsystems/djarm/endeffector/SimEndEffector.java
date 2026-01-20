package frc.robot.subsystems.djarm.endeffector;

import frc.robot.subsystems.djarm.DJArmConstants.EndEffectorState;

public class SimEndEffector implements EndEffectorIO {

    private EndEffectorState gripperState = EndEffectorState.OPEN;

    @Override
    public void setState(EndEffectorState targetGripperState) {
        gripperState = targetGripperState;
    }

    @Override
    public EndEffectorState getState() {
        return gripperState;
    }
}
