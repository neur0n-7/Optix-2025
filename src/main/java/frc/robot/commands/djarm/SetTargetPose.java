package frc.robot.commands.djarm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.djarm.DJArmSubsystem;
import frc.robot.subsystems.djarm.DJArmConstants.DJArmStoredPoses;

public class SetTargetPose extends Command {

    private final DJArmSubsystem djarm;
    private final DJArmStoredPoses pose;

    public SetTargetPose(DJArmSubsystem djarm, DJArmStoredPoses pose) {
        this.djarm = djarm;
        this.pose = pose;
        addRequirements(djarm);
    }

    @Override
    public void initialize() {
        djarm.setTargetPose(pose);
    }

    @Override
    public boolean isFinished() {
        return djarm.atTarget();
    }
}
