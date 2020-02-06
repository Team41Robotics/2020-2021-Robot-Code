package frc.robot;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Joystick;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;

public class Driving {
	
	private WPI_TalonSRX talonRF, talonRB, talonLF, talonLB;
	private TalonSRX test;
	private CANCoder leftEncoder, rightEncoder;

	private SpeedControllerGroup leftSpeedCG, rightSpeedCG;
	private DifferentialDrive difDrive;

	// For velocity drive
	private double leftSpeed = 0;
	private double rightSpeed = 0; 

	public Driving() {
		talonRF = new WPI_TalonSRX(PORTS.TALON_RF);
		talonRB = new WPI_TalonSRX(PORTS.TALON_RB);
		talonLF = new WPI_TalonSRX(PORTS.TALON_LF);
		talonLB = new WPI_TalonSRX(PORTS.TALON_LB);

		rightEncoder = new CANCoder(PORTS.TALON_RB);
		leftEncoder = new CANCoder(PORTS.TALON_LB);

		leftSpeedCG = new SpeedControllerGroup(talonLF, talonLB);
		rightSpeedCG = new SpeedControllerGroup(talonRF, talonRB);

		difDrive = new DifferentialDrive(leftSpeedCG, rightSpeedCG);
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

		double vl = leftEncoder.getVelocity();
		double vr = rightEncoder.getVelocity();

		if(vl < velocityLeft && leftSpeed < 1) { //Replace this with PID later?
			leftSpeed += .001;
		} 
		else if (vl > velocityLeft && leftSpeed > -1)
			leftSpeed -= .001;
		
		if(vr < velocityRight && rightSpeed < 1) {
			rightSpeed += .001;
		} 
		else if(vr > velocityRight && rightSpeed > -1)
			rightSpeed -= .001;

		difDrive.tankDrive(leftSpeed, rightSpeed);
	}

	public void controllerMove(Joystick controller) {
		double leftAxis = controller.getRawAxis(BUTTONS.GAMEPAD.LEFT_JOY_Y_AXIS);
		double rightAxis = controller.getRawAxis(BUTTONS.GAMEPAD.RIGHT_JOY_Y_AXIS);

		double speedMultiplier = 0.55;

		// System.out.println("Left: " + leftEncoder.getPosition() + " Right: " + rightEncoder.getPosition());
		difDrive.tankDrive(-leftAxis*speedMultiplier, -rightAxis*speedMultiplier);
		//if(leftAxis > 0) driveVelocity(0, .5); //.5 radians per second
	}
}