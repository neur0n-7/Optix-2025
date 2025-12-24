package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;

public class SetElevatorState extends Command {

    private final ElevatorSubsystem elevator;
    private final ElevatorStates target;

    public SetElevatorState(ElevatorSubsystem elevator, ElevatorStates state) {
        this.elevator = elevator;
        this.target = state;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.setTargetState(target);
    }

    @Override
    public boolean isFinished() {
        return elevator.atTarget();
    }
}
