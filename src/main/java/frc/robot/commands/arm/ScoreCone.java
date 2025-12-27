package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmConstants.ArmPositionStates;
import frc.robot.subsystems.arm.ArmConstants.CargoStates;
import frc.robot.subsystems.arm.ArmConstants.GripperStates;

public class ScoreCone extends SequentialCommandGroup {

    public ScoreCone(ArmSubsystem arm, ArmPositionStates scoreTargetState){
        
        addCommands(
            // stow > high
            new InstantCommand(() -> arm.setTargetPositionState(scoreTargetState)),
            new WaitUntilCommand(() -> arm.atPositionTarget()),

            // open gripper
            new InstantCommand(() -> arm.setGripperState(GripperStates.OPEN)),

            // set cargo to empty
            new InstantCommand(() -> arm.setCargoState(CargoStates.EMPTY)),

            // high > stow
            new InstantCommand(() -> arm.setTargetPositionState(ArmPositionStates.STOW)),
            new WaitUntilCommand(() -> arm.atPositionTarget())
        );
    }
}
