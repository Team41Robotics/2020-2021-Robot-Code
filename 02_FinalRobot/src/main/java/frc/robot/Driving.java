package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Driving {
	private final MotorType kBrushless = MotorType.kBrushless;
	
	private Joystick controller;
	private CANSparkMax sparkRF, sparkRB, sparkLF, sparkLB;

	// For velocity drive
	private double leftSpeed = 0;
	private double rightSpeed = 0;

	private final double maxAcc = 0.0075, maxDec = 0.015;

	public Driving() {
		controller = Robot.controller;
		sparkRF = new CANSparkMax(PORTS.SPARK_RF, kBrushless);
		sparkRB = new CANSparkMax(PORTS.SPARK_RB, kBrushless);
		sparkLF = new CANSparkMax(PORTS.SPARK_LF, kBrushless);
		sparkLB = new CANSparkMax(PORTS.SPARK_LB, kBrushless);
	}


	public void controllerMove() {
		double leftAxis = controller.getRawAxis(BUTTONS.GAMEPAD.L_JOY_Y_AXIS);
		double rightAxis = controller.getRawAxis(BUTTONS.GAMEPAD.R_JOY_Y_AXIS);
		
		final double deadband = 0.05;
		if(Math.abs(leftAxis) < deadband)
			leftAxis = 0;
		if(Math.abs(rightAxis) < deadband)
			rightAxis = 0;

		final double speedMultiplier = 0.5;
		leftAxis *= speedMultiplier;
		rightAxis *= speedMultiplier;

		// Emergency stop
		if(controller.getRawButtonPressed(BUTTONS.GAMEPAD.L_JOY_CLICK)) {
			leftSpeed = 0;
			rightSpeed = 0;
		}
			
		double tolerance = 0.05;
		if(Math.abs(leftSpeed - leftAxis) > tolerance) {
			double delta;
			double direction = 1;
			if(leftAxis < leftSpeed)
				direction = -1;
				
			if(Math.abs(leftSpeed) < Math.abs(leftAxis))
				delta = maxAcc;
			else
				delta = maxDec;
			leftSpeed += delta*direction;
		}
		else {
			leftSpeed = leftAxis;
		}

		if(Math.abs(rightSpeed - rightAxis) > tolerance) {
			double delta;
			double direction = 1;
			if(rightAxis < rightSpeed)
				direction = -1;

			if(Math.abs(rightSpeed) < Math.abs(rightAxis))
				delta = maxAcc;
			else
				delta = maxDec;
			rightSpeed += delta*direction;
		}
		else {
			rightSpeed = rightAxis;
		}

		// For safety
		if(leftSpeed > 1)
			leftSpeed = 1;
		else if(leftSpeed < -1)
			leftSpeed = -1;
		if(rightSpeed > 1)
			rightSpeed = 1;
		else if(rightSpeed < -1)
			rightSpeed = -1;
		
		drive();
		SmartDashboard.putNumber("left speed", leftSpeed);
		SmartDashboard.putNumber("right speed", rightSpeed);
		SmartDashboard.putNumber("left axis", leftAxis);
		SmartDashboard.putNumber("right axis", rightAxis);
	}

	private void drive() {
		double rightDriveSpeed = rightSpeed;
		// if(Math.abs(rightDriveSpeed) < 0.02)
		// 	rightDriveSpeed = 0;

		double leftDriveSpeed = leftSpeed;
		// if(Math.abs(leftDriveSpeed) < 0.02)
		// 	leftDriveSpeed = 0;

		sparkRF.set(-rightDriveSpeed);
		sparkRB.set(-rightDriveSpeed);
		sparkLF.set(leftDriveSpeed);
		sparkLB.set(leftDriveSpeed);
	}

	/**
	 * <a href="http://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf">Source Link</a>
	 * @param linear Linear velocity to drive at (in m/s)
	 * @param angular Angular velocity to rotate at (in rad/s)
	 */
	public void driveVelocity(double linear, double angular) {
		double wheelAxis = 0.5969; //meters, change to constant later
		double turnRadius = linear/angular; //v = r*omega

		double velocityLeft = angular*(turnRadius + wheelAxis/2);
		double velocityRight = angular*(turnRadius - wheelAxis/2);

		/* double vl = talonLB.getSelectedSensorVelocity();
		double vr = talonRB.getSelectedSensorVelocity();

		if(vl < velocityLeft && leftSpeed < 1) { //Replace this with PID later?
			leftSpeed += .001;
		} 
		else if (vl > velocityLeft && leftSpeed > -1)
			leftSpeed -= .001;
		
		if(vr < velocityRight && rightSpeed < 1) {
			rightSpeed += .001;
		} 
		else if(vr > velocityRight && rightSpeed > -1)
			rightSpeed -= .001; */

		drive();
	}

}