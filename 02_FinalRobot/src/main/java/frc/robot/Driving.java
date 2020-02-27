package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Driving {
	private final MotorType kBrushless = MotorType.kBrushless;
	
	private Joystick controller;
	private CANSparkMax sparkRF, sparkRB, sparkLF, sparkLB;

	// For velocity drive
	private double leftSpeed = 0;
	private double rightSpeed = 0; 

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

		double speedMultiplier = 0.25;

		drive(leftAxis*speedMultiplier, rightAxis*speedMultiplier);
	}

	private void drive(double leftSpeed, double rightSpeed) {
		sparkRF.set(-rightSpeed);
		sparkRB.set(-rightSpeed);
		sparkLF.set(leftSpeed);
		sparkLB.set(leftSpeed);
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

		drive(leftSpeed, rightSpeed);
	}

}