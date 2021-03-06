/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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
	public static Joystick driverstation = new Joystick(2);
	public static Joystick leftJoy = new Joystick(3);
	public static Joystick rightJoy = new Joystick(4);
	
	// public static Limelight lime;
	// public static Turret turret;
	// public static Hood hood;
	//public static Intake intake;
	// public static Driving drive;

	//Testing Purposes
	private final MotorType kBrushless = MotorType.kBrushless;
	private CANSparkMax spark;
	private TalonFX falconTalon;
	
	@Override
	public void robotInit() {
		// lime = new Limelight();
		// turret = new Turret();
		// hood = new Hood();
		// intake = new Intake();
		// drive = new Driving();

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
	
	}

	@Override
	public void teleopInit() {
		
	}

	@Override
	public void teleopPeriodic() {
		// lime.periodic();
		// turret.periodic();
		// if(useHood) hood.periodic();
		
		// drive.periodic();
	}

	@Override
	public void testInit() {
		spark = new CANSparkMax(0, kBrushless);
		// falconTalon = new TalonFX(0);
	}

	@Override
	public void testPeriodic() {
		// Get Joystick Axis
		double leftAxis = leftJoy.getRawAxis(BUTTONS.DRIVER_STATION.L_JOY_Y_AXIS);
		double rightAxis = rightJoy.getRawAxis(BUTTONS.DRIVER_STATION.R_JOY_Y_AXIS);

		// Set large deadband to stop robot from moving when it shouldn't be
		final double deadband = 0.15;
		if(Math.abs(leftAxis) < deadband)
			leftAxis = 0;
		if(Math.abs(rightAxis) < deadband)
			rightAxis = 0;
		
		// Set Motor Speed
		spark.set(leftAxis);
		// falconTalon.set(ControlMode.PercentOutput, leftAxis); 
	}

	@Override
	public void disabledInit() {
		System.out.println("Disabled");
	}
}
