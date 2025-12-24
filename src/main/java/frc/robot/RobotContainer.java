// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ExampleSubsystem;

// SWERVE
import frc.robot.commands.swerve.JoystickDrive;
import java.util.function.DoubleSupplier;

import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveSubsystem;

// DRIVE
import frc.robot.commands.drive.GoToDegrees;
import frc.robot.commands.elevator.SetElevatorState;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.SimElevatorMotor;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.RealNeoMotor;
import frc.robot.subsystems.SimNeoMotor;

public class RobotContainer {

	Boolean[] subsystemsEnabled = {false, false, true};

	// DRIVE
	private final DriveSubsystem m_DriveSubsystem;
	private final GoToDegrees m_GoTo90Degrees;
	private final GoToDegrees m_GoTo0Degrees;


	// SWERVE
	private final SwerveSubsystem m_SwerveSubsystem;
	private final JoystickDrive m_JoystickDrive;
	private final DoubleSupplier xSpeedSupplier;
	private final DoubleSupplier ySpeedSupplier;
	private final DoubleSupplier rotSpeedSupplier;

	// ELEVATOR V2
	private final ElevatorSubsystem m_ElevatorSubsystem2;
	private final SetElevatorState m_GoToElevatorHighest;
	private final SetElevatorState m_GoToElevatorLowest;
	private final SetElevatorState m_GoToElevatorMiddle;
	
	private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

	private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

	public RobotContainer() {
		
		if (RobotBase.isSimulation()){
			// DRIVE
			m_DriveSubsystem = new DriveSubsystem(new SimNeoMotor());

			// ELEVATOR V2
			m_ElevatorSubsystem2 = new ElevatorSubsystem(new SimElevatorMotor(), true);
			
			
		} else {
			// DRIVE
			m_DriveSubsystem = new DriveSubsystem(new RealNeoMotor(OperatorConstants.driveMotorCanId));

			// ELEVATOR V2
			m_ElevatorSubsystem2 = new ElevatorSubsystem(new SimElevatorMotor(), false);
		}

		// ELEVATOR V2 ////////////////////
		m_GoToElevatorLowest = new SetElevatorState(m_ElevatorSubsystem2, ElevatorConstants.ElevatorStates.LOWEST);
		m_GoToElevatorMiddle = new SetElevatorState(m_ElevatorSubsystem2, ElevatorConstants.ElevatorStates.MIDDLE);
		m_GoToElevatorHighest = new SetElevatorState(m_ElevatorSubsystem2, ElevatorConstants.ElevatorStates.HIGHEST);

		// DRIVE ////////////////////
		m_GoTo90Degrees = new GoToDegrees(m_DriveSubsystem, 90);
		m_GoTo0Degrees = new GoToDegrees(m_DriveSubsystem, 0);

		// SWERVE ////////////////////

		// Supplier lambdas
		// left joystick handles movement, right joystick handles turning
		
		xSpeedSupplier = () -> -MathUtil.applyDeadband(
			m_driverController.getLeftY(), 0.1) * SwerveConstants.ControlConstants.teleopMaxSpeedMetersPerSecond;

		ySpeedSupplier = () -> MathUtil.applyDeadband(
			m_driverController.getLeftX(), 0.1) * SwerveConstants.ControlConstants.teleopMaxSpeedMetersPerSecond;

		rotSpeedSupplier = () -> MathUtil.applyDeadband(
			m_driverController.getRightX(), 0.1) * SwerveConstants.ControlConstants.teleopMaxAngularSpeedRadPerSecond;
				
		m_SwerveSubsystem = new SwerveSubsystem(RobotBase.isReal());
		m_JoystickDrive = new JoystickDrive(
			m_SwerveSubsystem,
			xSpeedSupplier,
			ySpeedSupplier,
			rotSpeedSupplier
		);	
	 	

	
		// CONFIGURE BINDINGS ////////////////////
		configureBindings();

	}

	private void configureBindings() {
		/*
		 * The bindings are as follows:
		 *  - Left joystick to move robot w/ swerve
		 *  - Right joystick to rotate robot w/ swerve
		 * 
		 *  - X to go to elevator max
		 *  - Y to go to elevator min
		 *  - A to go to elevator mid
		 * 
		 * 	Drive (single motor):
		 *  - Left bumper to go to 0 degrees
		 *  - Right bumper to go to 90 degrees
		 */

		if (subsystemsEnabled[0]){
			// SWERVE
			System.out.println("SWERVE - Subsystem enabled");
			m_SwerveSubsystem.setDefaultCommand(m_JoystickDrive);
		};

		if (subsystemsEnabled[1]){
			// ELEVATOR V2
			System.out.println("ELEVATOR (V2) - Subsystem enabled");
			m_driverController.x().onTrue(m_GoToElevatorHighest);
			m_driverController.y().onTrue(m_GoToElevatorLowest);
			m_driverController.a().onTrue(m_GoToElevatorMiddle);
		}

		if (subsystemsEnabled[2]){
			// DRIVE
			System.out.println("DRIVE - Subsystem enabled");
			m_driverController.b().onTrue(m_GoTo0Degrees);
			m_driverController.b().onFalse(m_GoTo90Degrees);
		}
		
	}

	public Command getAutonomousCommand() {
		// An example command will be run in autonomous
		return Autos.exampleAuto(m_exampleSubsystem);
	}
  
}
