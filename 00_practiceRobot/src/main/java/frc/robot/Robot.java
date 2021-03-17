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
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.Paths;
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
	public static PurePursuit purePursuit;
		

	private Compressor comp;

	private NetworkTable realsense;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		if(!practiceRobot) {
			// lime = new Limelight();
			// turret = new Turret();
			// hood = new Hood();
			drive = new Driving();
			//intake = new Intake();
			purePursuit = new PurePursuit(Paths.competitionPath1);

			// comp = new Compressor(PORTS.PCM);
			// comp.start();
			// comp.stop();
		}

		// Start driver camera
		UsbCamera cam = CameraServer.getInstance().startAutomaticCapture();
		cam.setVideoMode(PixelFormat.kMJPEG, 80, 60, 60);

		realsense = NetworkTableInstance.getDefault().getTable("realsense");
	}

	@Override
	public void robotPeriodic() {

	}

	/**
	 * This function is run once each time the robot enters autonomous mode.
	 */
	@Override
	public void autonomousInit() {
		
		inAuton = true;
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		inAuton = true;

		// Not sure what the 2nd part of the block below does, was in original code
		/*
		// If we've reached our desired pos and are up to speed, we're ready to shoot
		if(turret.rampState == RAMP_STATE.NO_RAMP) {
			// intake.setAutonState(AUTON_STATE.SHOOT);
			// intake.setIntakeArmPos(true);
		}
		// lime.periodic();
		// turret.periodic();	
		// if(useHood) hood.periodic();
		// intake.periodic();

		*/

		//PURE PURSUIT
	  double xR = realsense.getEntry("xFieldRobot").getDouble(0);
      double yR = realsense.getEntry("yFieldRobot").getDouble(0);
	  double thetaR = realsense.getEntry("thetaFieldRobot").getDouble(0);
	  thetaR = thetaR/180*Math.PI;
	  
	  //System.out.println(xR+ " " + yR + " " + thetaR);
		
		purePursuit.calculateClosestPoint(xR, yR);
		purePursuit.calculateLookAhead(xR, yR, thetaR);
		double curvature = purePursuit.calcCurvatureToLookAhead(xR, yR, thetaR);
		//System.out.println(curvature);

		double maxV = purePursuit.getMaxVelocityAtClosestPoint();
		//System.out.println("maxV: " + maxV);
		System.out.println("omega:" + curvature*maxV);
		
		double LVel = maxV * (2 + curvature*Constants.DriveConstants.kTrackwidthMeters) /2;
		double RVel = maxV * (2 - curvature*Constants.DriveConstants.kTrackwidthMeters) /2;

		//System.out.println(LVel + " " + RVel);

		if(maxV == 0) drive.stopMotors();
		else drive.trackWheelVelocities(LVel, RVel);
		//drive.trackWheelVelocities(Constants.DriveConstants.kTrackwidthMeters/2*Math.PI/4, -Constants.DriveConstants.kTrackwidthMeters/2*Math.PI/4);
	}

	/**
	 * This function is called once each time the robot enters teleoperated mode.
	 */
	@Override
	public void teleopInit() {
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
		}
		
	}

	@Override
	public void testInit() {

	}

	@Override
	public void disabledInit() {
		System.out.println("Disabled");
	}
}
