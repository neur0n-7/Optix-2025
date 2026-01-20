// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

    // SWERVE
    public static final int kFrontLeftDriveMotorPort = 1;
    public static final int kFrontLeftTurningMotorPort = 2;
    public static final int kFrontRightDriveMotorPort = 3;
    public static final int kFrontRightTurningMotorPort = 4;
    public static final int kBackLeftDriveMotorPort = 5;
    public static final int kBackLeftTurningMotorPort = 6;
    public static final int kBackRightDriveMotorPort = 7;
    public static final int kBackRightTurningMotorPort = 8;

    // ELEVATOR
    public static final int elevatorMotorCanId = 23;

    // DRIVE (single motor basically)
    public static final int driveMotorCanId = 4;

    // ARM
    public static final int armMotorCanId = 20;
    public static final PneumaticsModuleType gripperModuleType = PneumaticsModuleType.CTREPCM;
    public static final int gripperChannel = 1;

    // DOUBLE JOINTED ARM
    public static final int shoulderJointCanId = 21;
    public static final int elbowJointCanId = 22;
    public static final PneumaticsModuleType endEffectorModuleType = PneumaticsModuleType.CTREPCM;
    public static final int endEffectorChannel = 2;

    

  }
}
