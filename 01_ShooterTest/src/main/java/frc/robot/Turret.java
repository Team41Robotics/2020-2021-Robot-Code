package frc.robot;

import java.text.DecimalFormat;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Turret {
	private TalonSRX intakeTalon;
	private TalonFX falconTalon;
	private CANSparkMax rotateSpark;
	private Encoder axleEncoder;
	private CANCoder falconEncoder;
	private CANEncoder rotateEncoder;
	private DigitalInput rotateLimit;
	private double shooterSpeed = 0, intakeSpeed = 0;
	private PIDController falconPID, rotatePID;
	private Vision vis;
	private DecimalFormat df;

	private enum RampState {NO_RAMP, UP, DOWN};
	private RampState rampState = RampState.NO_RAMP;
	private double rampPeriod = -1;
	private double targetSpeed = 0;
	private double startSpeed = 0;

	private boolean turretZeroed = false;	
	
	private final double maxSpeed = 0.95; // From 0 to 1
	private final boolean feedForward = true;
	private final boolean usePID = true;
	private final boolean useVision = true;
	
	// Constants
	private final double MAX_FALCON_ENCODER_RATE = 6_500; // Found experimentally
	private final double TURRET_ANGLE_MIN = -144; // Angle of the limit switch
	private final double TURRET_ANGLE_MAX = 60; // To stop the wires from breaking
	private final double FALCON_Kp = 1.2, FALCON_Ki = 0.2, FALCON_Kd = 0.0;
	private final double ROTATE_Kp = 0.065, ROTATE_Ki = 0.0075, ROTATE_Kd = 0.0175; // 0.065, 0.0075, 0.0175

	public Turret(Vision vis) {
		falconTalon = new TalonFX(PORTS.FALCON_TALON);
		intakeTalon = new TalonSRX(PORTS.INTAKE_TALON);
		rotateSpark = new CANSparkMax(PORTS.ROTATE_SPARK, MotorType.kBrushless);
		
		// falconEncoder = new CANCoder(PORTS.FALCON_TALON);
		// falconEncoder.setDistancePerPulse(1.0/2048.0 * 60); // Shows encoder data in gearRatio*rpm
		axleEncoder = new Encoder(PORTS.AXLE_ENCODER_A, PORTS.AXLE_ENCODER_B);
		axleEncoder.setDistancePerPulse(1.0/2048.0 * 60); // Shows encoder data in rpm
		rotateEncoder = rotateSpark.getEncoder();
		rotateEncoder.setPositionConversionFactor((1.0/7.0) * (1.0/5.0) * (16.0/132.0) * 360.0); // 1:7 gear ratio times 1:5 times 16:132 sprockets

		rotateLimit = new DigitalInput(PORTS.ROTATE_LIMIT);
		
		rotatePID = new PIDController(ROTATE_Kp, ROTATE_Ki, ROTATE_Kd);
		falconPID = new PIDController(FALCON_Kp, FALCON_Ki, FALCON_Kd);

		this.vis = vis;

		df = new DecimalFormat("0.##");

		turretInit();
	}

	private void turretInit() {
		// Factory Default all hardware to prevent unexpected behaviour
		falconTalon.configFactoryDefault();
		intakeTalon.configFactoryDefault();

		// Set Neutral Mode to Brake so Back-EMF doesn't break the robot
		falconTalon.setNeutralMode(NeutralMode.Brake);
		intakeTalon.setNeutralMode(NeutralMode.Brake);

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

		// Stop the shooter and intake using the left bumper
		if(controller.getRawButtonPressed(BUTTONS.GAMEPAD.L_BUMPER)) {
			// Immediately stop intake
			intakeSpeed = 0;
			// Ramp shooter down
			startSpeed = shooterSpeed;
			targetSpeed = 0;
			rampState = RampState.DOWN;
			setRampPeriod();
		}

		double falconOutputSpeed = shooterSpeed;
		if(usePID) {
			falconOutputSpeed = falconPID.calculate(
				axleEncoder.getRate()/MAX_FALCON_ENCODER_RATE,
				shooterSpeed
			);
			if(shooterSpeed == 0) {
				falconOutputSpeed = 0;
			}
			else if(feedForward) {
				falconOutputSpeed += shooterSpeed;
			}
		}
		falconTalon.set(ControlMode.PercentOutput, falconOutputSpeed);
		intakeTalon.set(ControlMode.PercentOutput, -intakeSpeed);

		SmartDashboard.putNumber("shooter encoder rate", axleEncoder.getRate());
		SmartDashboard.putNumber("falcon current", falconTalon.getSupplyCurrent());
		// SmartDashboard.putNumber("falcon encoder rate", falconEncoder.getVelocity());
		SmartDashboard.putNumber("rotate encoder", rotateEncoder.getPosition());

	}

	private void setShooterSpeed(Joystick controller) {
		double speedIncrement = 0.05;
		if(controller.getRawButton(BUTTONS.GAMEPAD.BACK_BUTTON)) // Fine adjustment
			speedIncrement = 0.005;

		//Change shooterSpeed using the x and y buttons
		//Y button increases shooterSpeed in the counterclockwise direction
		if(controller.getRawButtonPressed(BUTTONS.GAMEPAD.Y_BUTTON) && shooterSpeed <= maxSpeed) {
			shooterSpeed += speedIncrement;
			System.out.println("shooterSpeed: " + df.format(shooterSpeed*100.0) + "%");
		}
		//X button decreases shooterSpeed towards the clockwise direction
		if(controller.getRawButtonPressed(BUTTONS.GAMEPAD.X_BUTTON) && shooterSpeed >= -maxSpeed) {
			shooterSpeed -= speedIncrement;
			System.out.println("shooterSpeed: " + df.format(shooterSpeed*100.0) + "%");
		}

		// If D-Pad up button is pressed, adjust speed based on distance calculated by limelight
		if(controller.getPOV() == 0 && useVision) {
			startSpeed = shooterSpeed;
			targetSpeed = vis.getShooterSpeed();
			if(targetSpeed > maxSpeed) // Don't want to try to go too fast
				targetSpeed = maxSpeed;
			
			if(shooterSpeed < targetSpeed)
				rampState = RampState.UP;
			else
				rampState = RampState.DOWN;
			setRampPeriod();
			System.out.println("Setting speed to: " + df.format(targetSpeed*100) + "%");
		}

		if(controller.getRawButtonPressed(BUTTONS.GAMEPAD.R_BUMPER)) { // Start ramping up the speed
			if(targetSpeed < maxSpeed) {
				startSpeed = shooterSpeed;
				targetSpeed = maxSpeed;
				rampState = RampState.UP;
				setRampPeriod();
				System.out.println("Starting ramp up to " + Math.round(maxSpeed*100) + "% speed over " + rampPeriod + " seconds");
			}
		}


		if(rampState == RampState.DOWN) {
			if(shooterSpeed > targetSpeed) // If we have started the ramp down
				shooterSpeed += 0.02 * (targetSpeed - startSpeed) / rampPeriod; // 20 is because this iterates every 20 ms
			else {
				rampState = RampState.NO_RAMP;
				targetSpeed = 0;
				startSpeed = 0;
			}
		}
		else if(rampState == RampState.UP) {
			if(shooterSpeed < targetSpeed) // If we have started the ramp up
				shooterSpeed += 0.02 * (targetSpeed - startSpeed) / rampPeriod; // 20 is because this iterates every 20 ms
			else {
				rampState = RampState.NO_RAMP;
				targetSpeed = 0;
				startSpeed = 0;
			}
		}
	}
	private void setIntakeSpeed(Joystick controller) {
		//Change intakeSpeed using the A and B buttons
		//B button increases shooterSpeed in the counterclockwise direction
		if(controller.getRawButtonPressed(BUTTONS.GAMEPAD.B_BUTTON) && intakeSpeed <= 1.0) {
			intakeSpeed += 0.2;
			System.out.println("intakeSpeed: " + Math.round(intakeSpeed*100.0) + "%");
		}
		//A button decreases shooterSpeed towards the clockwise direction
		if(controller.getRawButtonPressed(BUTTONS.GAMEPAD.A_BUTTON) && intakeSpeed >= -1.0) {
			intakeSpeed -= 0.2;
			System.out.println("intakeSpeed: " + Math.round(intakeSpeed*100.0) + "%");
		}
	}

	private void setRampPeriod() {
		double time = Math.abs(shooterSpeed - targetSpeed)*10.0;
		 // Make ramp down slower than ramp up to protect motor
		if(rampState == RampState.DOWN)
			time *= 1.5;

		rampPeriod = time;
	}

	/**
	 * Sets the rotation speed of the turret
	 * @param controller
	 */
	private void rotate(Joystick controller) {
		if(!turretZeroed) { // Don't move if not zeroed yet
			zeroTurret();
			return;
		}

		if(!rotateLimit.get()) { // If limit is clicked, zero the encoder
			rotateEncoder.setPosition(TURRET_ANGLE_MIN);
		}

		double leftTrigger = controller.getRawAxis(BUTTONS.GAMEPAD.LEFT_TRIGGER_AXIS);
		double rightTrigger = controller.getRawAxis(BUTTONS.GAMEPAD.RIGHT_TRIGGER_AXIS);

		double speed = 0;
		double speedMultiplier = 0.20;
		if(leftTrigger > 0) { // Counterclockwise
			if(rotateEncoder.getPosition() < TURRET_ANGLE_MAX) {
				speed = leftTrigger*leftTrigger*leftTrigger*speedMultiplier;
			}
		}
		else if(rightTrigger > 0) { //Clockwise
			if(rotateLimit.get()) { // If limit is not activated
				speed = -rightTrigger*rightTrigger*rightTrigger*speedMultiplier;
			}
		}
		else if(vis.isTracking() && useVision) {
			double tolerance = .1; // In degrees
			double tx = vis.getTX();
			if(tx > tolerance) {
				// Since it's currently aimed to the left of the center
				// of the target, we want to adjust by moving clockwise
				if(rotateLimit.get()) {// Limit reads true when not clicked
					speed = rotatePID.calculate(0, -tx);
				}
			}
			else if(tx < -tolerance) {
				// Since it's currently aimed to the right of the center
				// of the target, we want to adjust by moving counterclockwise
				if(rotateEncoder.getPosition() < TURRET_ANGLE_MAX) {
					speed = rotatePID.calculate(0, -tx);
				}
			} else {
				rotatePID.reset(); //resets accumulated integral error when within threshold
			}
		}
		rotateSpark.set(speed);
	}

	private void zeroTurret() {
        if(!rotateLimit.get()) { // Limit reads false when clicked
			rotateEncoder.setPosition(TURRET_ANGLE_MIN);
            turretZeroed = true;
			rotateSpark.set(0);
			System.out.println("Turret zeroed");
        }
        else{
			rotateSpark.set(-0.2); // Slowly rotate clockwise ... until limit is hit
		}
	}
}