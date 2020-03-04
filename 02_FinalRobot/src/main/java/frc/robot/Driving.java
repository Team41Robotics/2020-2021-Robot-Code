package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Driving {
	private final MotorType kBrushless = MotorType.kBrushless;
	
	private Joystick leftJoy, rightJoy;
	private CANSparkMax sparkRF, sparkRB, sparkLF, sparkLB;
	private CANEncoder leftEnc, rightEnc;

	private double speedMultiplier = 0.5;
	private double leftSpeed = 0;
	private double rightSpeed = 0;
	private PIDController leftPid, rightPid;
	private boolean usePid = false;

	private final double maxAcc = 0.01, maxDec = 0.015;
	private final double L_Kp = 0.01, L_Ki = 0.0, L_Kd = 0.0;
	private final double R_Kp = 0.01, R_Ki = 0.0, R_Kd = 0.0;

	public Driving() {
		leftJoy = Robot.leftJoy;
		rightJoy = Robot.rightJoy;
		sparkRF = new CANSparkMax(PORTS.SPARK_RF, kBrushless);
		sparkRB = new CANSparkMax(PORTS.SPARK_RB, kBrushless);
		sparkLF = new CANSparkMax(PORTS.SPARK_LF, kBrushless);
		sparkLB = new CANSparkMax(PORTS.SPARK_LB, kBrushless);

		rightEnc = sparkRF.getEncoder();
		leftEnc = sparkLF.getEncoder();

		leftPid = new PIDController(L_Kp, L_Ki, L_Kd);
		rightPid = new PIDController(R_Kp, R_Ki, R_Kd);
	}


	public void controllerMove() {
		double leftAxis = leftJoy.getRawAxis(BUTTONS.DRIVER_STATION.L_JOY_Y_AXIS);
		double rightAxis = rightJoy.getRawAxis(BUTTONS.DRIVER_STATION.R_JOY_Y_AXIS);

		// Set large deadband to stop robot from moving when it shouldn't be
		final double deadband = 0.15;
		if(Math.abs(leftAxis) < deadband)
			leftAxis = 0;
		if(Math.abs(rightAxis) < deadband)
			rightAxis = 0;

		// Use speed multiplier to slow down driving. Default is 0.5
		final double speedMultiplierIncrement = 0.1;
		if(leftJoy.getRawButtonPressed(BUTTONS.DRIVER_STATION.L_JOY_BUTTON_RIGHT) && speedMultiplier >= speedMultiplierIncrement) {
			speedMultiplier -= speedMultiplierIncrement;
			System.out.println("Drivetrain Speed Multiplier -> " + speedMultiplier);
		}
		else if(rightJoy.getRawButtonPressed(BUTTONS.DRIVER_STATION.R_JOY_BUTTON_LEFT) && speedMultiplier <= 1 - speedMultiplierIncrement) {
			speedMultiplier += speedMultiplierIncrement;
			System.out.println("Drivetrain Speed Multiplier -> " + speedMultiplier);
		}
		leftAxis *= speedMultiplier;
		rightAxis *= speedMultiplier;

		// Emergency stop (just in case)
		if(leftJoy.getRawButtonPressed(BUTTONS.DRIVER_STATION.L_JOY_BUTTON_LEFT)) {
			leftSpeed = 0;
			rightSpeed = 0;
		}
		
		if(usePid) {
			leftSpeed = leftPid.calculate(leftAxis*speedMultiplier, leftEnc.getVelocity()); // Goal, current
			rightSpeed = rightPid.calculate(rightAxis*speedMultiplier, rightEnc.getVelocity()); // Goal, current
		}
		else {
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
		SmartDashboard.putNumber("left drivetrain encoder velocity", leftEnc.getVelocity());
		SmartDashboard.putNumber("right drivetrain encoder velocity", rightEnc.getVelocity());

	}

	private void drive() {
		double pow = 3.0;

		// leftDriveSpeed = leftJoy.getRawAxis(BUTTONS.DRIVER_STATION.L_JOY_Y_AXIS);
		// rightDriveSpeed = rightJoy.getRawAxis(BUTTONS.DRIVER_STATION.R_JOY_Y_AXIS);

		// if(cube) {
		// 	rightDriveSpeed = Math.pow(leftDriveSpeed, 3);
		// 	leftDriveSpeed = Math.pow(rightDriveSpeed, 3);
		// }

		sparkRF.set(-Math.pow(rightSpeed, pow));
		sparkRB.set(-Math.pow(rightSpeed, pow));
		sparkLF.set(Math.pow(leftSpeed, pow));
		sparkLB.set(Math.pow(leftSpeed, pow));
	}

	/**
	 * <a href="http://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf">Source Link</a>
	 * @param linear Linear velocity to drive at (in m/s)
	 * @param angular Angular velocity to rotate at (in rad/s)
	 */
	public void driveVelocity(double linear, double angular) {
		/* double wheelAxis = 0.5969; //meters, change to constant later
		double turnRadius = linear/angular; //v = r*omega

		double velocityLeft = angular*(turnRadius + wheelAxis/2);
		double velocityRight = angular*(turnRadius - wheelAxis/2);

		double vl = talonLB.getSelectedSensorVelocity();
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