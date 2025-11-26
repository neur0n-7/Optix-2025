package frc.robot.subsystems.swerve;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubsystem extends SubsystemBase{
    private final SwerveModule frontLeft = new SwerveModule(
        SwerveConstants.kFrontLeftDriveMotorPort,
        SwerveConstants.kFrontLeftTurningMotorPort,
        SwerveConstants.kFrontLeftDriveEncoderReversed,
        SwerveConstants.kFrontLeftTurningEncoderReversed,
        SwerveConstants.kFrontLeftDriveAbsoluteEncoderPort,
        SwerveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
        SwerveConstants.kFrontLeftDriveAbsoluteEncoderReversed
    );

    private final SwerveModule frontRight = new SwerveModule(
        SwerveConstants.kFrontRightDriveMotorPort,
        SwerveConstants.kFrontRightTurningMotorPort,
        SwerveConstants.kFrontRightDriveEncoderReversed,
        SwerveConstants.kFrontRightTurningEncoderReversed,
        SwerveConstants.kFrontRightDriveAbsoluteEncoderPort,
        SwerveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
        SwerveConstants.kFrontRightDriveAbsoluteEncoderReversed
    );

    private final SwerveModule backLeft = new SwerveModule(
        SwerveConstants.kBackLeftDriveMotorPort,
        SwerveConstants.kBackLeftTurningMotorPort,
        SwerveConstants.kBackLeftDriveEncoderReversed,
        SwerveConstants.kBackLeftTurningEncoderReversed,
        SwerveConstants.kBackLeftDriveAbsoluteEncoderPort,
        SwerveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
        SwerveConstants.kBackLeftDriveAbsoluteEncoderReversed
    );

    private final SwerveModule backRight = new SwerveModule(
        SwerveConstants.kBackRightDriveMotorPort,
        SwerveConstants.kBackRightTurningMotorPort,
        SwerveConstants.kBackRightDriveEncoderReversed,
        SwerveConstants.kBackRightTurningEncoderReversed,
        SwerveConstants.kBackRightDriveAbsoluteEncoderPort,
        SwerveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
        SwerveConstants.kBackRightDriveAbsoluteEncoderReversed
    );


}
