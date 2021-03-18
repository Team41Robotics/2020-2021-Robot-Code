/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
	public final static boolean useHood = false;
	
	public static Joystick driverstation = new Joystick(2);
	public static Joystick leftJoy = new Joystick(3);
	public static Joystick rightJoy = new Joystick(4);
	
	public static Limelight lime;
	public static Turret turret;
	// public static Hood hood;
	public static Intake intake;
	public static Driving drive;
	public static Indexer indexer;
	private Compressor comp;

	//Testing Purposes
	/*
	private CANSparkMax rotateSpark;
	private double rotateSpeed;
	private DigitalInput rotateLimitLeft, rotateLimitCenter, rotateLimitRight;
	private CANEncoder rotateEncoder;
	*/

	
	@Override
	public void robotInit() {
		lime = new Limelight();
		turret = new Turret();
		// hood = new Hood();
		intake = new Intake();
		indexer = new Indexer();
		drive = new Driving();
		
		/*
		rotateSpark = new CANSparkMax(PORTS.ROTATE_SPARK, MotorType.kBrushless);
		
		rotateEncoder = rotateSpark.getEncoder();
		// Show encoder data in degrees of rotation (with 0 being straight forward)
		rotateEncoder.setPositionConversionFactor((1.0/7.0) * (16.0/132.0) * 360.0); // 1:7 gear ratio times 16:132 sprockets

		rotateLimitLeft = new DigitalInput(PORTS.ROTATE_LIMIT_LEFT);
		rotateLimitCenter = new DigitalInput(PORTS.ROTATE_LIMIT_CENTER);
		rotateLimitRight = new DigitalInput(PORTS.ROTATE_LIMIT_RIGHT);
		*/

		comp = new Compressor(PORTS.PCM);
		comp.clearAllPCMStickyFaults();
		comp.setClosedLoopControl(true);
		//System.out.println("Compressor Start");
		comp.start();
		
		// Start driver camera
		// UsbCamera cam = CameraServer.getInstance().startAutomaticCapture();
		// cam.setVideoMode(PixelFormat.kMJPEG, 80, 60, 60);
	}

	@Override
	public void robotPeriodic() {

	}

	@Override
	public void autonomousInit() {

	}

	@Override
	public void autonomousPeriodic() {
		drive.trackWheelVelocities(0.5, 0.5);
	}

	@Override
	public void teleopInit() {	
		// rotateSpeed = 0;
	}

	@Override
	public void teleopPeriodic() {
		lime.periodic();
		turret.periodic();
		// if(useHood) hood.periodic();
		drive.periodic();
		intake.periodic();
		indexer.periodic();

		/*
		System.out.println("Left Toggle: " + driverstation.getRawButton(BUTTONS.DRIVER_STATION.TOGGLE_SWITCH_L));
		System.out.println("Middle Toggle: " + driverstation.getRawButton(BUTTONS.DRIVER_STATION.TOGGLE_SWITCH_M));
		System.out.println("Right Toggle: " + driverstation.getRawButton(BUTTONS.DRIVER_STATION.TOGGLE_SWITCH_R));
		System.out.println("Left Rocker Up: " + driverstation.getRawButton(BUTTONS.DRIVER_STATION.ROCKER_L_UP));
		System.out.println("Left Rocker Down: " + driverstation.getRawButton(BUTTONS.DRIVER_STATION.ROCKER_L_DOWN));
		System.out.println("Middle Rocker Up: " + driverstation.getRawButton(BUTTONS.DRIVER_STATION.ROCKER_M_UP));
		System.out.println("Middle Rocker Down: " + driverstation.getRawButton(BUTTONS.DRIVER_STATION.ROCKER_M_DOWN));
		System.out.println("Right Rocker Up: " + driverstation.getRawButton(BUTTONS.DRIVER_STATION.ROCKER_R_UP));
		System.out.println("Right Rocker Down: " + driverstation.getRawButton(BUTTONS.DRIVER_STATION.ROCKER_R_DOWN));
		*/

		
	}
	

	@Override
	public void testInit() {
		
	}

	@Override
	public void testPeriodic() {
	}

	@Override
	public void disabledInit() {
		System.out.println("Disabled");
	}
}
