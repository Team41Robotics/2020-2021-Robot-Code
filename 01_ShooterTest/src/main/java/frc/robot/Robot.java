/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/**
 * TODO List:
 * 1) Solve pi realsense boot issue
 * 2) Test realsense coordinate system
 * 3) Test autonomous movement
 * 4) Transfer code to final robot
 * 5) Collect shooter data for final robot
 * 6) Boot pi into CLI mode
 * 7) Shooter scanning using realsesne coord system                                   
 */



package frc.robot;

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
	private final Joystick controller = new Joystick(1);
	private final Joystick extraJoy = new Joystick(5);
	
	private Limelight lime;
	private Turret turret;
	private Hood hood;
	private Driving drive;
	private Music music;

	public final boolean useHood = true;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		lime = new Limelight(useHood);
		turret = new Turret(lime);
		if(useHood) hood = new Hood(lime);
		drive = new Driving();
		music = new Music();
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
		lime.runLimelight(controller);
		turret.controllerMove(controller);
		if(useHood) hood.controllerMove(extraJoy);
		drive.controllerMove(controller);
		
		// music.playMusic();

	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
}
