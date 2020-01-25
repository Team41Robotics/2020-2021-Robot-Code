package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Turret {
	private TalonSRX shooterTalon1, shooterTalon2, intakeTalon;
	private CANSparkMax rotateSpark;
	private Encoder axleEncoder, shooterEncoder1, shooterEncoder2;
	private double shooterSpeed, intakeSpeed;
	private PIDController pid1, pid2;

	private boolean rampUp = false;
	private final double maxSpeed = 0.5; // From 0 to 1
	private final double rampUpPeriod = 5.0; // In seconds
	private final boolean feedForward = true;
	private final boolean limitCurrent = false;

	private final double MAX_ENCODER_RATE = 17_000; // This is in 4*rpm
	private final double Kp = 30.0, Ki = 1.0, Kd = 0.0;

	public Turret() {
		// shooterTalon1 = new TalonSRX(PORTS.SHOOTER_TALON_1);
		shooterTalon2 = new TalonSRX(PORTS.SHOOTER_TALON_2);
		intakeTalon = new TalonSRX(PORTS.INTAKE_TALON);
		rotateSpark = new CANSparkMax(PORTS.ROTATE_SPARK, MotorType.kBrushless);

		axleEncoder = new Encoder(PORTS.AXLE_ENCODER_A, PORTS.AXLE_ENCODER_B);
		axleEncoder.setDistancePerPulse(1.0/2048.0 * 60); // Shows encoder data in rpm
		// shooterEncoder1 = new Encoder(PORTS.SHOOTER_ENCODER_1_A, PORTS.SHOOTER_ENCODER_1_B);
		// shooterEncoder1.setDistancePerPulse(1.0/1024.0 * 60); // Shows encoder data in 4*rpm
		shooterEncoder2 = new Encoder(PORTS.SHOOTER_ENCODER_2_A, PORTS.SHOOTER_ENCODER_2_B);
		shooterEncoder2.setDistancePerPulse(1.0/1024.0 * 60); // Shows encoder data in 4*rpm

		// pid1 = new PIDController(Kp, Ki, Kd);
		pid2 = new PIDController(Kp, Ki, Kd);


		turretInit();
	}

	private void turretInit() {
		shooterSpeed = 0.0;
		intakeSpeed = 0.0;

		// Have to set this current limit in phoenix tuner for motor #1 !!!
		// shooterTalon1.enableCurrentLimit(limitCurrent);
		shooterTalon2.enableCurrentLimit(limitCurrent);

		// Factory Default all hardware to prevent unexpected behaviour
		// shooterTalon1.configFactoryDefault();
		shooterTalon2.configFactoryDefault();
		intakeTalon.configFactoryDefault();
		// Set Neutral Mode to Brake so Back-EMF doesn't break the robot
		// shooterTalon1.setNeutralMode(NeutralMode.Brake);
		shooterTalon2.setNeutralMode(NeutralMode.Brake);
		intakeTalon.setNeutralMode(NeutralMode.Brake);
		// Set peak outputs to 1.0
		// shooterTalon1.configPeakOutputForward(+1.0, CONSTANTS.kTimeoutMs);
		// shooterTalon1.configPeakOutputReverse(-1.0, CONSTANTS.kTimeoutMs);
		shooterTalon2.configPeakOutputForward(+1.0, CONSTANTS.kTimeoutMs);
		shooterTalon2.configPeakOutputReverse(-1.0, CONSTANTS.kTimeoutMs);
		intakeTalon.configPeakOutputForward(+1.0, CONSTANTS.kTimeoutMs);
		intakeTalon.configPeakOutputReverse(-1.0, CONSTANTS.kTimeoutMs);
		// Quad Mode, primary device, 30 ms timeout
		/* shooterTalon1.configSelectedFeedbackSensor(	FeedbackDevice.CTRE_MagEncoder_Relative,
													CONSTANTS.PID_PRIMARY,
													CONSTANTS.kTimeoutMs);
		shooterTalon2.configSelectedFeedbackSensor(	FeedbackDevice.CTRE_MagEncoder_Relative,
													CONSTANTS.PID_PRIMARY,
													CONSTANTS.kTimeoutMs); */
	}

	public void controllerMove(Joystick controller) {
		shoot(controller);
		rotate(controller);
	}

	/**
	 * Sets the speed of the shooter
	 * @param controller
	 */
	private void shoot(Joystick controller) {
		setShooterSpeed(controller);
		setIntakeSpeed(controller);

		//Stop the motors in the shooter using the left bumper
		if(controller.getRawButtonPressed(BUTTONS.L_BUMPER)){
			shooterSpeed = 0;
			intakeSpeed = 0;
			rampUp = false;
		}

		// shooterTalon1.set(ControlMode.PercentOutput, shooterSpeed); 
		// shooterTalon2.set(ControlMode.PercentOutput, shooterSpeed);
		// double shooterPIDSpeed1 = pid1.calculate(
		// 	shooterEncoder1.getRate()/MAX_ENCODER_RATE,
		// 	shooterSpeed
		// );
		double shooterPIDSpeed2 = pid2.calculate(
			shooterEncoder2.getRate()/MAX_ENCODER_RATE,
			shooterSpeed
		);
		// shooterTalon1.set(ControlMode.PercentOutput, shooterPIDSpeed1);
		double outputSpeed2 = shooterPIDSpeed2;
		if(feedForward)
			outputSpeed2 += shooterSpeed;
		shooterTalon2.set(ControlMode.PercentOutput, outputSpeed2);
		intakeTalon.set(ControlMode.PercentOutput, intakeSpeed);

		SmartDashboard.putNumber("shooter encoder rate", axleEncoder.getRate());
		// SmartDashboard.putNumber("shooter talon 1 current", shooterTalon1.getStatorCurrent());
		SmartDashboard.putNumber("shooter talon 2 current", shooterTalon2.getStatorCurrent());
		// SmartDashboard.putNumber("shooter talon 1 encoder", shooterEncoder1.getRate());
		SmartDashboard.putNumber("shooter talon 2 encoder", shooterEncoder2.getRate());
		// SmartDashboard.putNumber("shooter talon 1 encoder error", pid1.getPositionError());
		SmartDashboard.putNumber("shooter talon 2 encoder error", pid2.getPositionError());
	}

	private void setShooterSpeed(Joystick controller) {
		//Change shooterSpeed using the x and y buttons
		//Y button increases shooterSpeed in the counterclockwise direction
		if(controller.getRawButtonPressed(BUTTONS.Y_BUTTON) && shooterSpeed <= 1.0) {
			shooterSpeed += 0.05;
			System.out.println("shooterSpeed: " + Math.round(shooterSpeed*100.0) + "%");
		}
		//X button decreases shooterSpeed towards the clockwise direction
		if(controller.getRawButtonPressed(BUTTONS.X_BUTTON) && shooterSpeed >= -1.0) {
			shooterSpeed -= 0.05;
			System.out.println("shooterSpeed: " + Math.round(shooterSpeed*100.0) + "%");
		}

		// if(controller.getRawButtonPressed(BUTTONS.R_BUMPER)) {
		// 	shooterSpeed = 0.5;
		// 	System.out.println("shooterSpeed: " + Math.round(shooterSpeed*100.0) + "%");
		// }
		if(controller.getRawButtonPressed(BUTTONS.R_BUMPER)) { // Start ramping up the speed
			if(rampUpPeriod == 0)
				shooterSpeed = maxSpeed;
			else
				rampUp = true;
			System.out.println("Starting ramp up to " + Math.round(maxSpeed*100) + "% speed");
		}
		if(rampUp && shooterSpeed < maxSpeed) { // If we have started the ramp up
			shooterSpeed += 20/(rampUpPeriod*1000) * maxSpeed; // 20 is because this iterates every 20 ms
		}
	}
	private void setIntakeSpeed(Joystick controller) {
		//Change intakeSpeed using the A and B buttons
		//B button increases shooterSpeed in the counterclockwise direction
		if(controller.getRawButtonPressed(BUTTONS.B_BUTTON) && intakeSpeed <= 1.0) {
			intakeSpeed += 0.1;
			System.out.println("intakeSpeed: " + Math.round(intakeSpeed*100.0) + "%");
		}
		//A button decreases shooterSpeed towards the clockwise direction
		if(controller.getRawButtonPressed(BUTTONS.A_BUTTON) && intakeSpeed >= -1.0) {
			intakeSpeed -= 0.1;
			System.out.println("intakeSpeed: " + Math.round(intakeSpeed*100.0) + "%");
		}
	}

	/**
	 * Sets the rotation speed of the turret
	 * @param controller
	 */
	private void rotate(Joystick controller) {
		double leftTrigger = controller.getRawAxis(BUTTONS.LEFT_TRIGGER_AXIS);
		double rightTrigger = controller.getRawAxis(BUTTONS.RIGHT_TRIGGER_AXIS);

		double speed = 0;
		double speedMultiplier = 0.1;
		if(leftTrigger > 0) { // Counterclockwise
			// if(turretEnc.getRaw() < TURRET_ENC_MAX) {
				speed = leftTrigger*leftTrigger*leftTrigger*speedMultiplier;
				// motorDrive = pid.calculate(encoderValAdjusted, leftTrigger);
			// }
		}
		else if(rightTrigger > 0) { //Clockwise
			// if(!turretZeroed || turretEnc.getRaw() > TURRET_ENC_MIN) {
				speed = -rightTrigger*rightTrigger*rightTrigger*speedMultiplier;
				// motorDrive = pid.calculate(encoderValAdjusted, -rightTrigger);
			// }
		}
		rotateSpark.set(speed);
	}
}

class CONSTANTS {
	public final static int kTimeoutMs = 30,
	PID_PRIMARY = 0,
	REMOTE_1 = 1;
}