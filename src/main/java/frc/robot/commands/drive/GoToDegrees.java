
package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;

public class GoToDegrees extends Command {

    private final DriveSubsystem m_subsystem;
    private final double degrees;

    public GoToDegrees(DriveSubsystem subsystem, double degrees) {
        this.m_subsystem = subsystem;
        this.degrees = degrees;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_subsystem.setTargetDegrees(degrees);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}