package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Driving {
	private final MotorType kBrushless = MotorType.kBrushless;
	
	private Joystick leftJoy, rightJoy;
	private CANSparkMax sparkLF, sparkLB, sparkRF, sparkRB;
	private CANEncoder leftEnc, rightEnc;

	private double speedMultiplier = 1.0; // Starts robot at 50% speed for manual control
	private double leftSpeed = 0;
	private double rightSpeed = 0;
	private PIDController leftPid, rightPid, leftPPPid, rightPPPid;
	private boolean feedForward = true;
	private boolean usePid = true;
	private boolean useCurve = true;

	private final double MAX_DRIVE_SPEED = 5_500;
	private final double L_Kp = 0.03, L_Ki = 0, L_Kd = 0.004; // 0.3 0 0.003
	private final double R_Kp = 0.03, R_Ki = 0, R_Kd = 0.004; // 0.3 0 0.003

	private final double PPL_Kp = 0.02, PPL_Ki = 0, PPL_Kd = 0.004; // 0.3 0 0.003
	private final double PPR_Kp = 0.02, PPR_Ki = 0, PPR_Kd = 0.004;


	// PURE PURSUIT STUFF

	public Driving() {
		leftJoy = Robot.leftJoy;
		rightJoy = Robot.rightJoy;
		sparkLF = new CANSparkMax(PORTS.SPARK_LF, kBrushless);
		sparkLB = new CANSparkMax(PORTS.SPARK_LB, kBrushless);
		sparkRF = new CANSparkMax(PORTS.SPARK_RF, kBrushless);
		sparkRB = new CANSparkMax(PORTS.SPARK_RB, kBrushless);

		leftEnc = sparkLF.getEncoder();
		rightEnc = sparkRB.getEncoder();

		leftEnc.setPosition(0);
		rightEnc.setPosition(0);

		leftEnc.setVelocityConversionFactor(DriveConstants.kEncoderDistancePerPulse);
		rightEnc.setVelocityConversionFactor(DriveConstants.kEncoderDistancePerPulse);

		leftPPPid = new PIDController(PPL_Kp, PPL_Ki, PPL_Kd);
		rightPPPid = new PIDController(PPR_Kp, PPR_Ki, PPR_Kd);

		leftPid = new PIDController(L_Kp, L_Ki, L_Kd);
		rightPid = new PIDController(R_Kp, R_Ki, R_Kd);
	}

	public void trackWheelVelocities(double LVel, double RVel) {

		double leftVelocity = leftEnc.getVelocity();
		double rightVelocity = rightEnc.getVelocity();

		leftSpeed += leftPPPid.calculate(-leftVelocity, LVel);
		rightSpeed += rightPPPid.calculate(rightVelocity, RVel);

		drive();
		
		SmartDashboard.putNumber("left speed", leftSpeed);
		SmartDashboard.putNumber("right speed", rightSpeed);		
		SmartDashboard.putNumber("left drivetrain encoder velocity", -leftEnc.getVelocity());
		SmartDashboard.putNumber("right drivetrain encoder velocity", rightEnc.getVelocity());
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
		/*if(leftJoy.getRawButton(BUTTONS.DRIVER_STATION.L_JOY_BUTTON_RIGHT) && rightJoy.getRawButton(BUTTONS.DRIVER_STATION.R_JOY_BUTTON_LEFT)) {
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
			final double speedMultiplierIncrement = 1.0;
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
		}*/

		if(useCurve){
			double a = 0.8;
			double b = 0.0;
			leftAxis = b + (1-b)*(a*Math.pow(leftAxis, 3) + (1-a)*leftAxis);
			rightAxis = b + (1-b)*(a*Math.pow(rightAxis, 3) + (1-a)*rightAxis);
		}

		System.out.println("Left Axis: " + leftAxis);
		System.out.println("Right Axis: " + rightAxis);
		
		
		if(usePid) {
			double leftVelocity = leftEnc.getVelocity();
			double rightVelocity = rightEnc.getVelocity();

			leftSpeed += leftPid.calculate(-leftVelocity, -leftAxis*2);
			rightSpeed += rightPid.calculate(rightVelocity, -rightAxis*2);
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

		SmartDashboard.putNumber("left axis", leftAxis);
		SmartDashboard.putNumber("right axis", rightAxis);

		SmartDashboard.putNumber("left speed", leftSpeed);
		SmartDashboard.putNumber("right speed", rightSpeed);		
		SmartDashboard.putNumber("left drivetrain encoder velocity", -leftEnc.getVelocity());
		SmartDashboard.putNumber("right drivetrain encoder velocity", rightEnc.getVelocity());
	}

	private void drive() {
		sparkRF.set(rightSpeed);
		sparkRB.set(rightSpeed);
		sparkLF.set(-leftSpeed);
		sparkLB.set(-leftSpeed);
	}
}