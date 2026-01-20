package frc.robot.commands.djarm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.djarm.DJArmSubsystem;
import frc.robot.subsystems.djarm.DJArmConstants.EndEffectorState;

public class ToggleEndEffector extends Command {

    private final DJArmSubsystem djarm;

    public ToggleEndEffector(DJArmSubsystem djarm) {
        this.djarm = djarm;
        addRequirements(djarm);
    }

    @Override
    public void initialize() {
        EndEffectorState currentState = djarm.getEndEffectorState();
        djarm.setEndEffectorState(currentState.isClosed ? EndEffectorState.OPEN : EndEffectorState.CLOSED);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
