package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;

public class GoToElevatorHeight extends Command {

    private final ElevatorSubsystem elevator;
    private final ElevatorStates target;

    public GoToElevatorHeight(ElevatorSubsystem elevator, ElevatorStates state) {
        this.elevator = elevator;
        this.target = state;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.setTarget(target.position);
    }

    @Override
    public boolean isFinished() {
        return elevator.atSetpoint();
    }
}
