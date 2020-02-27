package frc.robot;

import java.text.DecimalFormat;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Turret {
	private Joystick controller;
	private TalonFX falconTalon;
	private CANSparkMax rotateSpark;
	private Encoder shaftEncoder;
	private CANEncoder rotateEncoder;
	private DigitalInput rotateLimitLeft, rotateLimitCenter, rotateLimitRight;
	private double shooterSpeed = 0;
	private PIDController falconPID, rotatePID;
	private Limelight lime;
	private DecimalFormat df;
	// private Orchestra orchestra;

	private enum Direction {CLOCKWISE, COUNTERCLOCKWISE};
	private enum RampState {NO_RAMP, UP, DOWN};
	private RampState rampState = RampState.NO_RAMP;
	private double rampPeriod = -1;
	private double targetSpeed = 0;
	private double startSpeed = 0;

	private boolean turretZeroed = false;	
	
	private final double maxSpeed = 0.975; // From 0 to 1
	private final boolean feedForward = true;
	private final boolean usePID = true;
	private final boolean useVision = true;
	
	// Constants
	private final double MAX_FALCON_ENCODER_RATE = 6_500; // Found experimentally
	private final double TURRET_ANGLE_CLOCKWISE_MAX = -113; // Max clockwise angle
	private final double TURRET_ANGLE_COUNTERCLOCKWISE_MAX = 200; // Max counterclockwise angle
	private final double FALCON_Kp = 1.2, FALCON_Ki = 0.2, FALCON_Kd = 0.0;
	private final double ROTATE_Kp = 0.065, ROTATE_Ki = 0.0075, ROTATE_Kd = 0.0175; // 0.065, 0.0075, 0.0175

	public Turret() {
		controller = Robot.controller;

		falconTalon = new TalonFX(PORTS.FALCON_TALON);
		rotateSpark = new CANSparkMax(PORTS.ROTATE_SPARK, MotorType.kBrushless);
		
		falconTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
		// falconTalon.configPulseWidthPeriod_EdgesPerRot(2048, 0);
		// falconEncoder.setDistancePerPulse(1.0/2048.0 * 60); // Shows encoder data in gearRatio*rpm
		shaftEncoder = new Encoder(PORTS.SHAFT_ENCODER_A, PORTS.SHAFT_ENCODER_B);
		shaftEncoder.setDistancePerPulse(1.0/2048.0 * 60); // Shows encoder data in rpm
		rotateEncoder = rotateSpark.getEncoder();
		// Show encoder data in degrees of rotation (with 0 being straight forward)
		rotateEncoder.setPositionConversionFactor((1.0/7.0) * (1.0/5.0) * (16.0/132.0) * 360.0); // 1:7 gear ratio times 1:5 times 16:132 sprockets

		rotateLimitLeft = new DigitalInput(PORTS.ROTATE_LIMIT_LEFT);
		rotateLimitCenter = new DigitalInput(PORTS.ROTATE_LIMIT_CENTER);
		rotateLimitRight = new DigitalInput(PORTS.ROTATE_LIMIT_RIGHT);
		
		rotatePID = new PIDController(ROTATE_Kp, ROTATE_Ki, ROTATE_Kd);
		falconPID = new PIDController(FALCON_Kp, FALCON_Ki, FALCON_Kd);

		this.lime = Robot.lime;

		df = new DecimalFormat("0.##");

		// orchestra = new Orchestra();

		turretInit();
	}

	private void turretInit() {
		// Factory Default all hardware to prevent unexpected behaviour
		falconTalon.configFactoryDefault();

		// Set Neutral Mode to Brake so Back-EMF doesn't break the robot
		falconTalon.setNeutralMode(NeutralMode.Brake);

		// orchestra.addInstrument(falconTalon);
		// orchestra.loadMusic("song1.chrp");
	}

	public void controllerMove() {
		shoot();
		rotate();
		playMusic();
	}

	/**
	 * Set the shooter speed based on controller input
	 * @param controller
	 */
	private void shoot() {
		setShooterSpeed();
		// Stop the shooter and intake using the left bumper
		if(controller.getRawButtonPressed(BUTTONS.GAMEPAD.L_BUMPER)) {
			// Ramp shooter down
			startSpeed = shooterSpeed;
			targetSpeed = 0;
			rampState = RampState.DOWN;
			setRampPeriod();
			System.out.println("Shooter Speed (ramp) -> 0%");
			System.out.println("Intake Speed -> 0%");
		}

		double falconOutputSpeed = shooterSpeed;
		if(usePID) {
			falconOutputSpeed = falconPID.calculate(
				shaftEncoder.getRate()/MAX_FALCON_ENCODER_RATE,
				shooterSpeed
			);
			if(shooterSpeed == 0) {
				falconOutputSpeed = 0;
			}
			else if(feedForward) {
				falconOutputSpeed += shooterSpeed;
			}
		}
		falconTalon.set(ControlMode.PercentOutput, -falconOutputSpeed);

		SmartDashboard.putNumber("shooter encoder rate", shaftEncoder.getRate());
		SmartDashboard.putNumber("falcon current", falconTalon.getSupplyCurrent());
		SmartDashboard.putNumber("rotate encoder", rotateEncoder.getPosition());

		SmartDashboard.putBoolean("rotate limit left", rotateLimitLeft.get());
		SmartDashboard.putBoolean("rotate limit center", rotateLimitCenter.get());
		SmartDashboard.putBoolean("rotate limit right", rotateLimitRight.get());
	}

	private void setShooterSpeed() {
		double speedIncrement = 0.05;
		if(controller.getRawButton(BUTTONS.GAMEPAD.BACK_BUTTON)) // Fine adjustment
			speedIncrement = 0.005;

		// Change shooterSpeed using the x and y buttons
		// Y button increases shooterSpeed in the shooting direction
		if(controller.getRawButtonPressed(BUTTONS.GAMEPAD.Y_BUTTON) && shooterSpeed < maxSpeed) {
			shooterSpeed += speedIncrement;
			System.out.println("Shooter Speed -> " + df.format(shooterSpeed*100.0) + "%");
		}
		// X button decreases shooterSpeed towards the reverse direction
		if(controller.getRawButtonPressed(BUTTONS.GAMEPAD.X_BUTTON) && shooterSpeed > 0) {
			shooterSpeed -= speedIncrement;
			System.out.println("Shooter Speed -> " + df.format(shooterSpeed*100.0) + "%");
		}

		// Adjust speed based on distance calculated by limelight
		if(controller.getRawButtonPressed(BUTTONS.GAMEPAD.R_BUMPER) && useVision) {
			startSpeed = shooterSpeed;
			targetSpeed = lime.getShooterSpeed();
			if(targetSpeed > maxSpeed) // Don't want to try to go too fast
				targetSpeed = maxSpeed;
			
			if(shooterSpeed < targetSpeed)
				rampState = RampState.UP;
			else
				rampState = RampState.DOWN;
			setRampPeriod();
			System.out.println("Shooter Speed (ramp) -> " + df.format(targetSpeed*100) + "%");
		}


		if(rampState == RampState.DOWN) {
			if(shooterSpeed > targetSpeed) // If we have started the ramp down
				shooterSpeed += 0.02 * (targetSpeed - startSpeed) / rampPeriod; // 20 is because this iterates every 20 ms
			else {
				rampState = RampState.NO_RAMP;
				targetSpeed = 0;
				startSpeed = 0;
				System.out.println("Target shooter speed achieved");
			}
		}
		else if(rampState == RampState.UP) {
			if(shooterSpeed < targetSpeed) // If we have started the ramp up
				shooterSpeed += 0.02 * (targetSpeed - startSpeed) / rampPeriod; // 20 is because this iterates every 20 ms
			else {
				rampState = RampState.NO_RAMP;
				targetSpeed = 0;
				startSpeed = 0;
				System.out.println("Target shooter speed achieved");
			}
		}
	}

	/**
	 * Sets the period that the shooter will ramp up over
	 * based on the desired change in speed
	 */
	private void setRampPeriod() {
		// Make time to ramp 10x the change in speed
		// Ex: 0.1 (10%) -> 0.6 (60%) is a 0.5 (50%)
		// change, so it will take 5.0 seconds to ramp up
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
	private void rotate() {
		// if(!turretZeroed) { // Don't move if not zeroed yet
		// 	zeroTurret();
		// 	return;
		// }

		// If limit is clicked, zero the encoder
		// If ONLY home limit is clicked - false when clicked
		if(getHomeLimit()) {
			rotateEncoder.setPosition(0);
		}
		// If max extension clockwise limit is clicked
        else if(getRotateLimit(Direction.CLOCKWISE)) {
			rotateEncoder.setPosition(TURRET_ANGLE_CLOCKWISE_MAX);
		}
		// If max extension counterclockwise limit is clicked
        else if(getRotateLimit(Direction.COUNTERCLOCKWISE)) {
			rotateEncoder.setPosition(TURRET_ANGLE_COUNTERCLOCKWISE_MAX);
        }

		double leftTrigger = controller.getRawAxis(BUTTONS.GAMEPAD.LEFT_TRIGGER_AXIS);
		double rightTrigger = controller.getRawAxis(BUTTONS.GAMEPAD.RIGHT_TRIGGER_AXIS);

		double speed = 0;
		double speedMultiplier = 0.20;
		if(leftTrigger > 0) { // Counterclockwise
			// if(rotateEncoder.getPosition() < TURRET_ANGLE_COUNTERCLOCKWISE_MAX) {
			if(!getRotateLimit(Direction.COUNTERCLOCKWISE)) {
				speed = leftTrigger*leftTrigger*leftTrigger*speedMultiplier;
			}
		}
		else if(rightTrigger > 0) { //Clockwise
			if(!getRotateLimit(Direction.CLOCKWISE)) { // If limit is not activated
				speed = -rightTrigger*rightTrigger*rightTrigger*speedMultiplier;
			}
		}
		else if(lime.isTracking() && useVision) {
			double tolerance = .5; // In degrees
			double tx = lime.getTX();
			if(tx > tolerance) {
				// Since it's currently aimed to the left of the center
				// of the target, we want to adjust by moving clockwise
				if(!getRotateLimit(Direction.CLOCKWISE)) { // If limit is not activated
					speed = rotatePID.calculate(0, -tx);
				}
			}
			else if(tx < -tolerance) {
				// Since it's currently aimed to the right of the center
				// of the target, we want to adjust by moving counterclockwise
				// if(rotateEncoder.getPosition() < TURRET_ANGLE_COUNTERCLOCKWISE_MAX) {
				if(!getRotateLimit(Direction.COUNTERCLOCKWISE)) {
					speed = rotatePID.calculate(0, -tx);
				}
			} else {
				rotatePID.reset(); //resets accumulated integral error when within threshold
			}
		}
		rotateSpark.set(speed);
	}

	private void zeroTurret() {
        if(getHomeLimit()) { // If home limit is clicked
			rotateEncoder.setPosition(0);
            turretZeroed = true;
			rotateSpark.set(0);
			System.out.println("Turret Zeroed");
        }
        else if(getRotateLimit(Direction.COUNTERCLOCKWISE)) { // If max extension limit is clicked
			rotateEncoder.setPosition(TURRET_ANGLE_CLOCKWISE_MAX);
            turretZeroed = true;
			rotateSpark.set(0);
			System.out.println("Turret Zeroed");
        }
        else {
			rotateSpark.set(0.1); // Slowly rotate clockwise ... until limit is hit
		}
	}

	private boolean getHomeLimit() {
		return !rotateLimitCenter.get() && rotateLimitLeft.get() && rotateLimitRight.get();
	}

	/**
	 * Checks if both limits are activated in a certain direction
	 * @return True if they both are clicked
	 */
	private boolean getRotateLimit(Direction dir) {
		if(dir == Direction.CLOCKWISE)
			return !rotateLimitCenter.get() && !rotateLimitRight.get();
		// If rotating counterclockwise
		return !rotateLimitCenter.get() && !rotateLimitLeft.get();
	}

	private void playMusic(){
		// if(controller.getRawButtonPressed(BUTTONS.GAMEPAD.R_JOY_CLICK) && shooterSpeed == 0.0)
		// 	orchestra.play();
	}
}