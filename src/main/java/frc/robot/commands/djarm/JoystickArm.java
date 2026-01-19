
package frc.robot.commands.djarm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.djarm.DJArmSubsystem;

public class JoystickArm extends Command {
    private final DJArmSubsystem djarm;
    private final DoubleSupplier xTargetSupplier;
    private final DoubleSupplier yTargetSupplier;

    public JoystickArm(
            DJArmSubsystem djarm,
            DoubleSupplier xTargetSupplier,
            DoubleSupplier yTargetSupplier
    ) {
        this.djarm = djarm;
        this.xTargetSupplier = xTargetSupplier;
        this.yTargetSupplier = yTargetSupplier;

        addRequirements(djarm);
    }

    @Override
    public void execute() {
        djarm.shiftTargetPose(xTargetSupplier.getAsDouble(), yTargetSupplier.getAsDouble());
    }
}
