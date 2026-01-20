package frc.robot.subsystems.djarm.endeffector;

import frc.robot.subsystems.djarm.DJArmConstants.EndEffectorState;

public interface EndEffectorIO {

    void setState(EndEffectorState targetGripperState);

    EndEffectorState getState();
}