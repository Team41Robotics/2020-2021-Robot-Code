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

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
	public final static boolean useHood = false;

	public static Joystick controller = new Joystick(1);
	public static Joystick extraJoy = new Joystick(5);
	
	public static final Limelight lime = new Limelight();
	public static final Turret turret = new Turret();
	public static final Hood hood = new Hood();
	public static final Driving drive = new Driving();
	public static final Intake intake = new Intake();
	public static final Climbing climb = new Climbing();

	private Compressor comp;


	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		comp = new Compressor(PORTS.PCM);
	}

	/**
	 * This function is run once each time the robot enters autonomous mode.
	 */
	@Override
	public void autonomousInit() {
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {

		
	}

	/**
	 * This function is called once each time the robot enters teleoperated mode.
	 */
	@Override
	public void teleopInit() {
		//System.out.println("Teleop initated");
		
	}

	/**
	 * This function is called periodically during teleoperated mode.
	 */
	@Override
	public void teleopPeriodic() {
		lime.runLimelight();
		turret.controllerMove();
		if(useHood) hood.controllerMove();
		drive.controllerMove();
		intake.controllerMove();
		// climb.controllerMove();

		comp.start();
	}
}
