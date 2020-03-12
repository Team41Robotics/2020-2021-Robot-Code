package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Driving {
	private final MotorType kBrushless = MotorType.kBrushless;
	
	private Joystick driverstation, leftJoy, rightJoy;
	private CANSparkMax sparkLF, sparkLB, sparkRF, sparkRB;
	private CANEncoder leftEnc, rightEnc;

	private double speedMultiplier = 0.7; // Starts robot at 50% speed for manual control
	private double leftSpeed = 0;
	private double rightSpeed = 0;
	private PIDController leftPid, rightPid;
	private boolean feedForward = true;
	private boolean usePid = true;

	private final double MAX_DRIVE_SPEED = 5_500;
	// Old - Too jerky
	// private final double L_Kp = 1.0, L_Ki = 0.05, L_Kd = 0.1;
	// private final double R_Kp = 1.0, R_Ki = 0.05, R_Kd = 0.1;
	private final double L_Kp = 0.5, L_Ki = 0.025, L_Kd = 0.05;
	private final double R_Kp = 0.5, R_Ki = 0.025, R_Kd = 0.05;


	public Driving() {
		driverstation = Robot.driverstation;
		leftJoy = Robot.leftJoy;
		rightJoy = Robot.rightJoy;
		sparkLF = new CANSparkMax(PORTS.SPARK_LF, kBrushless);
		sparkLB = new CANSparkMax(PORTS.SPARK_LB, kBrushless);
		sparkRF = new CANSparkMax(PORTS.SPARK_RF, kBrushless);
		sparkRB = new CANSparkMax(PORTS.SPARK_RB, kBrushless);

		leftEnc = sparkLF.getEncoder();
		rightEnc = sparkRF.getEncoder();

		leftPid = new PIDController(L_Kp, L_Ki, L_Kd);
		rightPid = new PIDController(R_Kp, R_Ki, R_Kd);
	}


	public void periodic() {
		double leftAxis = leftJoy.getRawAxis(BUTTONS.DRIVER_STATION.L_JOY_Y_AXIS);
		double rightAxis = rightJoy.getRawAxis(BUTTONS.DRIVER_STATION.R_JOY_Y_AXIS);

		// Set large deadband to stop robot from moving when it shouldn't be
		final double deadband = 0.15;
		if(Math.abs(leftAxis) < deadband)
			leftAxis = 0;
		if(Math.abs(rightAxis) < deadband)
			rightAxis = 0;


		// This slows down driving (useful for intaking balls since that requires slow speeds)
		if(leftJoy.getRawButton(BUTTONS.DRIVER_STATION.L_JOY_BUTTON_RIGHT) && rightJoy.getRawButton(BUTTONS.DRIVER_STATION.R_JOY_BUTTON_LEFT)) {
			leftAxis *= 0.45;
			rightAxis *= 0.45;
		}
		// This speeds up driving
		else if(leftJoy.getRawButton(BUTTONS.DRIVER_STATION.L_JOY_BUTTON_DOWN) && rightJoy.getRawButton(BUTTONS.DRIVER_STATION.R_JOY_BUTTON_DOWN)) {
			leftAxis *= 1.0;
			rightAxis *= 1.0;
		}
		else {
			// Use speed multiplier to slow down driving. Default is 1.0
			final double speedMultiplierIncrement = 0.1;
			if(leftJoy.getRawButtonPressed(BUTTONS.DRIVER_STATION.L_JOY_BUTTON_LEFT) && speedMultiplier >= speedMultiplierIncrement) {
				speedMultiplier -= speedMultiplierIncrement;
				System.out.println("Drivetrain Speed Multiplier -> " + speedMultiplier);
			}
			else if(rightJoy.getRawButtonPressed(BUTTONS.DRIVER_STATION.R_JOY_BUTTON_RIGHT) && speedMultiplier <= 1 - speedMultiplierIncrement) {
				speedMultiplier += speedMultiplierIncrement;
				System.out.println("Drivetrain Speed Multiplier -> " + speedMultiplier);
			}
			leftAxis *= speedMultiplier;
			rightAxis *= speedMultiplier;
		}
		
		if(usePid) {
			leftSpeed = leftPid.calculate(leftEnc.getVelocity()/MAX_DRIVE_SPEED, leftAxis); // Goal, current
			rightSpeed = rightPid.calculate(-rightEnc.getVelocity()/MAX_DRIVE_SPEED, rightAxis); // Goal, current
			if(feedForward) {
				leftSpeed += leftAxis;
				rightSpeed += rightAxis;
			}
		}
		else {
			leftSpeed = leftAxis;
			rightSpeed = rightAxis;
		}

		// For safety
		if(leftSpeed > 1) leftSpeed = 1;
		else if(leftSpeed < -1) leftSpeed = -1;
		if(rightSpeed > 1) rightSpeed = 1;
		else if(rightSpeed < -1) rightSpeed = -1;
		
		drive();

		SmartDashboard.putNumber("left speed", leftSpeed);
		SmartDashboard.putNumber("right speed", rightSpeed);
		SmartDashboard.putNumber("left axis", leftAxis);
		SmartDashboard.putNumber("right axis", rightAxis);
		SmartDashboard.putNumber("left drivetrain encoder velocity", leftEnc.getVelocity()/MAX_DRIVE_SPEED);
		SmartDashboard.putNumber("right drivetrain encoder velocity", rightEnc.getVelocity()/MAX_DRIVE_SPEED);
	}

	private void drive() {
		double pow = 3.0;

		sparkRF.set(-Math.pow(rightSpeed, pow));
		sparkRB.set(-Math.pow(rightSpeed, pow));
		sparkLF.set(Math.pow(leftSpeed, pow));
		sparkLB.set(Math.pow(leftSpeed, pow));
	}
}