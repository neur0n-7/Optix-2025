package frc.robot.commands.djarm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.djarm.DJArmSubsystem;
import frc.robot.subsystems.djarm.DJArmConstants.DJArmStoredPoses;
import frc.robot.subsystems.djarm.DJArmConstants.EndEffectorState;

public class ScoreHigh extends SequentialCommandGroup {

    public ScoreHigh(DJArmSubsystem djarm) {

        addRequirements(djarm);

        addCommands(
            new SequentialCommandGroup(
                    new InstantCommand(() -> djarm.setTargetPose(DJArmStoredPoses.HIGH.pose)),
                    new WaitUntilCommand(() -> djarm.atTarget()),
                    new InstantCommand(() -> djarm.setEndEffectorState(EndEffectorState.OPEN)),
                    new InstantCommand(() -> djarm.setTargetPose(DJArmStoredPoses.STOW.pose)),
                    new WaitUntilCommand(() -> djarm.atTarget())
            )
        );
    }
}
