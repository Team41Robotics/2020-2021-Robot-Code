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
	public static boolean inAuton = true;

	public static Joystick controller = new Joystick(1);
	public static Joystick extraJoy = new Joystick(5);
	public static Joystick driverstation = new Joystick(2);
	public static Joystick leftJoy = new Joystick(3);
	public static Joystick rightJoy = new Joystick(4);
	
	public static Limelight lime;
	public static Turret turret;
	public static Hood hood;
	public static Driving drive;

	private Compressor comp;

	@Override
	public void robotInit() {
		lime = new Limelight();
		turret = new Turret();
		hood = new Hood();
		drive = new Driving();

		comp = new Compressor(PORTS.PCM);
		comp.start();
		// comp.stop();

		// Start driver camera
		UsbCamera cam = CameraServer.getInstance().startAutomaticCapture();
		cam.setVideoMode(PixelFormat.kMJPEG, 80, 60, 60);
	}

	@Override
	public void robotPeriodic() {

	}

	@Override
	public void autonomousInit() {
		inAuton = true;
	}

	@Override
	public void autonomousPeriodic() {

	}

	@Override
	public void teleopInit() {
		inAuton = false;
	}

	@Override
	public void teleopPeriodic() {
		lime.periodic();
		turret.periodic();
		if(useHood) hood.periodic();
		drive.periodic();
	}

	@Override
	public void testInit() {

	}

	@Override
	public void disabledInit() {
		System.out.println("Disabled");
	}
}
