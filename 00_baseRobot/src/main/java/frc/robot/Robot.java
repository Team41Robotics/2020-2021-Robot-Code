/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/**
 * CONTROLS:
 * Gamepad Y = + shooter speed
 * Gamepad X = - shooter speed
 * Gamepad B = + shooter intake speed
 * Gamepad A = - shooter intake speed
 * Gamepad Left Trigger = Turn turret clockwise
 * Gamepad Right Trigger = Turn turret ccw
 * Gamepad Left Bumper = Stop shooter intake and ramp down shooter
 * Gamepad Right Bumper = Use limelight for speed calcs
 * Gamepad Back Button = Change speed increment from .05 to .005
 * Gamepad Start Button = Turn on limelight (lights and tracking)
 * Gamepad Left JS (Y axis) = Control robot left wheels
 * Gamepad Right JS (Y axis) = Control robot right wheels
 * 
 * Big Joystick Trigger = Toggle hood auto adjust
 * Big Joystick Right Handle Button = Move hood up
 * Big Joystick Left Handle Button = Move hood down
 * Big Joystick Up Handle Button = + Hood move increment
 * Big Joystick Down Handle Button = - Hood move increment 
 */
package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Intake.AUTON_STATE;
import frc.robot.Turret.RAMP_STATE;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
	public final static boolean useHood = true;
	public final static boolean practiceRobot = false;
	public static boolean inAuton = true;

	private Command m_autonomousCommand;
	private RobotContainer m_robotContainer;

	public static Joystick controller = new Joystick(1);
	public static Joystick extraJoy = new Joystick(5);
	public static Joystick driverstation = new Joystick(2);
	public static Joystick leftJoy = new Joystick(3);
	public static Joystick rightJoy = new Joystick(4);
	
	public static Limelight lime;
	public static Turret turret;
	public static Hood hood;
	public static Driving drive;
	public static Intake intake;
	public static Climbing climb;

	private Compressor comp;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		if(!practiceRobot) {
			lime = new Limelight();
			turret = new Turret();
			hood = new Hood();
			drive = new Driving();
			intake = new Intake();
			climb = new Climbing();

			comp = new Compressor(PORTS.PCM);
			comp.start();
			// comp.stop();
		}

		m_robotContainer = new RobotContainer();
		// m_robotContainer.m_robotDrive.navx.reset();

		// Start driver camera
		UsbCamera cam = CameraServer.getInstance().startAutomaticCapture();
		cam.setVideoMode(PixelFormat.kMJPEG, 80, 60, 60);
	}

	@Override
	public void robotPeriodic() {
		// Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
		// commands, running already-scheduled commands, removing finished or interrupted commands,
		// and running subsystem periodic() methods.  This must be called from the robot's periodic
		// block in order for anything in the Command-based framework to work.
		CommandScheduler.getInstance().run();

		Pose2d pose = m_robotContainer.m_robotDrive.getPose();
		SmartDashboard.putNumber("Pos x", pose.getTranslation().getX());
		SmartDashboard.putNumber("Pos y", pose.getTranslation().getY());
		SmartDashboard.putNumber("Pos angle (deg)", pose.getRotation().getDegrees());
		SmartDashboard.putNumber("Distance to target pose", m_robotContainer.getDistanceToTarget());
		SmartDashboard.putNumber("Yaw error", m_robotContainer.getYawError());
	}

	/**
	 * This function is run once each time the robot enters autonomous mode.
	 */
	@Override
	public void autonomousInit() {
		m_autonomousCommand = m_robotContainer.getAutonomousCommand();

		/*
		 * String autoSelected = SmartDashboard.getString("Auto Selector",
		 * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
		 * = new MyAutoCommand(); break; case "Default Auto": default:
		 * autonomousCommand = new ExampleCommand(); break; }
		 */

		// schedule the autonomous command (example)
		if (m_autonomousCommand != null) {
			m_autonomousCommand.schedule();
		}
		inAuton = true;
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		inAuton = true;
		// if (m_autonomousCommand.isFinished()) {
		if(m_robotContainer.atTargetPose()) {
			System.out.println("Ready to shoot");
			// Send a velocity of zero
			m_robotContainer.m_robotDrive.sendVelocity(0, 0);

			if(!practiceRobot) {
				// If we've reached our desired pos and are up to speed, we're ready to shoot
				if(turret.rampState == RAMP_STATE.NO_RAMP) {
					// intake.setAutonState(AUTON_STATE.SHOOT);
					// intake.setIntakeArmPos(true);
				}
			}
		}
		if(!practiceRobot) {
			// lime.periodic();
			// turret.periodic();
			// if(useHood) hood.periodic();
			// intake.periodic();
		}
	}

	/**
	 * This function is called once each time the robot enters teleoperated mode.
	 */
	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
		inAuton = false;
	}

	/**
	 * This function is called periodically during teleoperated mode.
	 */
	@Override
	public void teleopPeriodic() {
		inAuton = false;
		if(!practiceRobot) {// defaults to command based movement using arcade drive if using practice robot
			lime.periodic();
			turret.periodic();
			if(useHood) hood.periodic();
			intake.periodic();
			drive.periodic();
			climb.periodic();
		}

		// System.out.println(m_robotContainer.m_robotDrive.getPose());
	}

	@Override
	public void testInit() {
		// Cancels all running commands at the start of test mode.
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void disabledInit() {
		System.out.println("Disabled");
	}
}
