package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmConstants.ArmPositionStates;
import frc.robot.subsystems.arm.ArmConstants.GripperStates;
import frc.robot.subsystems.arm.ArmConstants.CargoStates;

public class IntakeCone extends SequentialCommandGroup {

    public IntakeCone(ArmSubsystem arm) {
        addCommands(
            // stow > intake
            new InstantCommand(() -> arm.setTargetPositionState(ArmPositionStates.INTAKE)),
            new WaitUntilCommand(() -> arm.atPositionTarget()),

            // close gripper
            new InstantCommand(() -> arm.setGripperState(GripperStates.CLOSED)),

            // set cargo to loaded
            new InstantCommand(() -> arm.setCargoState(CargoStates.LOADED)),

            // high > stow
            new InstantCommand(() -> arm.setTargetPositionState(ArmPositionStates.STOW)),
            new WaitUntilCommand(() -> arm.atPositionTarget())
        );
    }
}
