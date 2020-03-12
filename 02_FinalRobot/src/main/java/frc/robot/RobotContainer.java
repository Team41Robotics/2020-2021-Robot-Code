/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PAutoConstants;
import frc.robot.Constants.PDriveConstants;
import frc.robot.subsystems.DriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems
	public final DriveSubsystem m_robotDrive = new DriveSubsystem();

	// The driver's controller
 	private final Joystick leftJoy = Robot.leftJoy;
	private final Joystick rightJoy = Robot.rightJoy;
	 
	private final boolean practiceRobot = Robot.practiceRobot;

	private RamseteCommand ramseteCommand;
	private Trajectory exampleTrajectory;
	private TrajectoryConfig config;

	private Pose2d targetPose = new Pose2d(11, 2, new Rotation2d(0));

	/**
	 * The container for the robot.  Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		// Configure the button bindings
		configureButtonBindings();

		// Configure default commands
		// Set the default drive command to split-stick arcade drive
		if(practiceRobot)
			m_robotDrive.setDefaultCommand(
				// A split-stick arcade command, with forward/backward controlled by the left
				// hand, and turning controlled by the right.
				new RunCommand(() -> m_robotDrive
					.arcadeDrive(
						leftJoy.getRawAxis(BUTTONS.DRIVER_STATION.L_JOY_Y_AXIS)*-1,
						rightJoy.getRawAxis(BUTTONS.DRIVER_STATION.R_JOY_X_AXIS)),
					m_robotDrive
				)
			);
	}

	/**
	 * Use this method to define your button->command mappings.  Buttons can be created by
	 * instantiating a {@link GenericHID} or one of its subclasses ({@link
	 * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
	 * {@link JoystickButton}.
	 */
	private void configureButtonBindings() {
		// Drive at half speed when the right joy right button is held
		new JoystickButton(rightJoy, BUTTONS.DRIVER_STATION.R_JOY_BUTTON_RIGHT)
			.whenPressed(() -> m_robotDrive.setMaxOutput(0.05))
			.whenReleased(() -> m_robotDrive.setMaxOutput(0.1));
	}


	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// Create a voltage constraint to ensure we don't accelerate too fast
		if(practiceRobot) {
			var autoVoltageConstraint =
				new DifferentialDriveVoltageConstraint(
					new SimpleMotorFeedforward(
						DriveConstants.ksVolts,
						DriveConstants.kvVoltSecondsPerMeter,
						DriveConstants.kaVoltSecondsSquaredPerMeter
					),
					DriveConstants.kDriveKinematics,
					10
				);

			// Create config for trajectory
			config =
				new TrajectoryConfig(
						PAutoConstants.kMaxSpeedMetersPerSecond,
						PAutoConstants.kMaxAccelerationMetersPerSecondSquared
					)
					// Add kinematics to ensure max speed is actually obeyed
					.setKinematics(PDriveConstants.kDriveKinematics)
					// Apply the voltage constraint
					.addConstraint(autoVoltageConstraint);

			// An example trajectory to follow.  All units in meters.
			exampleTrajectory = TrajectoryGenerator.generateTrajectory(
				// Start position
				new Pose2d(0, 0, new Rotation2d(0)), // 0 degrees is +x
				// Pass through these two interior waypoints, making an 's' curve path
				List.of(
					// new Translation2d(1, 1),
					// new Translation2d(2, -1)
					// new Translation2d(3, 0)
				),
				// End Position
				new Pose2d(1, 0, new Rotation2d(0)),
				// Pass config
				config
			);

			ramseteCommand = new RamseteCommand(
				exampleTrajectory,
				m_robotDrive::getPose,
				new RamseteController(PAutoConstants.kRamseteB, PAutoConstants.kRamseteZeta),
				new SimpleMotorFeedforward(
					PDriveConstants.ksVolts,
					PDriveConstants.kvVoltSecondsPerMeter,
					PDriveConstants.kaVoltSecondsSquaredPerMeter
				),
				PDriveConstants.kDriveKinematics,
				m_robotDrive::getWheelSpeeds,
				new PIDController(PDriveConstants.kPDriveVel, 0, 0),
				new PIDController(PDriveConstants.kPDriveVel, 0, 0),
				// RamseteCommand passes volts to the callback
				m_robotDrive::tankDriveVolts,
				m_robotDrive
			);
		}
		else {
			var autoVoltageConstraint =
				new DifferentialDriveVoltageConstraint(
					new SimpleMotorFeedforward(
						DriveConstants.ksVolts,
						DriveConstants.kvVoltSecondsPerMeter,
						DriveConstants.kaVoltSecondsSquaredPerMeter
					),
					DriveConstants.kDriveKinematics,
					10
				);

			// Create config for trajectory
			config =
				new TrajectoryConfig(
						AutoConstants.kMaxSpeedMetersPerSecond,
						AutoConstants.kMaxAccelerationMetersPerSecondSquared
					)
					// Add kinematics to ensure max speed is actually obeyed
					.setKinematics(DriveConstants.kDriveKinematics)
					// Apply the voltage constraint
					.addConstraint(autoVoltageConstraint);

			// An example trajectory to follow.  All units in meters.
			exampleTrajectory = TrajectoryGenerator.generateTrajectory(
				// Start position
				new Pose2d(10, 2, new Rotation2d(0)), // 0 degrees is +x
				// Pass through these two interior waypoints, making an 's' curve path
				List.of(
					// new Translation2d(1, 1),
					// new Translation2d(2, -1)
					// new Translation2d(3, 0)
				),
				// End Position
				targetPose,
				// Pass config
				config
			);

			ramseteCommand = new RamseteCommand(
				exampleTrajectory,
				m_robotDrive::getPose,
				new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
				new SimpleMotorFeedforward(
					DriveConstants.ksVolts,
					DriveConstants.kvVoltSecondsPerMeter,
					DriveConstants.kaVoltSecondsSquaredPerMeter
				),
				DriveConstants.kDriveKinematics,
				m_robotDrive::getWheelSpeeds,
				new PIDController(DriveConstants.kPDriveVel, 0, 0),
				new PIDController(DriveConstants.kPDriveVel, 0, 0),
				// RamseteCommand passes volts to the callback
				m_robotDrive::tankDriveVolts,
				m_robotDrive
			);
		}
		// Run path following command, then stop at the end.
		return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
	}

	public Pose2d getTargetPose() {
		return targetPose;
	}

	public double getTargetX() {
		return targetPose.getTranslation().getX();
	}
	public double getTargetY() {
		return targetPose.getTranslation().getY();
	}
	/**
	 * 
	 * @return target angle in degrees
	 */
	public double getTargetAngle() {
		return targetPose.getRotation().getDegrees();
	}
	public double getDistanceToTarget() {
		double x = m_robotDrive.getPose().getTranslation().getX();
		double y = m_robotDrive.getPose().getTranslation().getY();
		return Math.sqrt(Math.pow(getTargetX() - x, 2) + Math.pow(getTargetY() - y, 2));
	}
	/**
	 * 
	 * @return error in degrees
	 */
	public double getYawError() {
		double angle = m_robotDrive.getPose().getRotation().getDegrees();
		double diff = Math.abs(getTargetAngle() - angle);
		double otherDiff = 360 - diff;
		return (diff < otherDiff) ? diff : otherDiff;
	}
	public boolean atTargetPose() {
		double distTolerance = 0.1; // 10cm
		double yawTolerance = 1; // In degrees

		// If we have set a target, x will be positive
		boolean setTarget = targetPose.getTranslation().getX() >= 0;

		return setTarget &&
				Math.abs(getDistanceToTarget()) < distTolerance &&
				Math.abs(getYawError()) < yawTolerance;
	}
}