package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Turret {
	private Joystick driverstation;
	private TalonFX falconTalon;
	private CANSparkMax rotateSpark;
	private CANEncoder rotateEncoder;
	private DigitalInput rotateLimitLeft, rotateLimitCenter, rotateLimitRight;
	private double shooterSpeed = 0;
	private PIDController falconPID, rotatePID;
	private Limelight lime;

	private enum Direction {CLOCKWISE, COUNTERCLOCKWISE};
	public enum RAMP_STATE {NO_RAMP, UP, DOWN};
	public RAMP_STATE rampState = RAMP_STATE.NO_RAMP;
	private double rampPeriod = -1;
	private double targetSpeed = 0;
	private double startSpeed = 0;

	private boolean turretZeroed = false;
	
	private final double maxSpeed = 0.975; // From 0 to 1
	private final boolean feedForward = true;
	private final boolean usePID = true;
	private final boolean autoRotate = true;
	
	// Constants
	private final double MAX_FALCON_ENCODER_RATE = 20_000;
	private final double TURRET_ANGLE_CLOCKWISE_MAX = -83.117; // Max clockwise angle
	private final double TURRET_ANGLE_COUNTERCLOCKWISE_MAX = 68; // Max counterclockwise angle
	private final double FALCON_Kp = 1.2, FALCON_Ki = 0.2, FALCON_Kd = 0.0;
	private final double ROTATE_Kp = 0.025, ROTATE_Ki = 0.0075, ROTATE_Kd = 0.0; // 0.065, 0.0075, 0.0175

	public Turret() {
		driverstation = Robot.driverstation;

		falconTalon = new TalonFX(PORTS.SHOOTER_FALCON);
		rotateSpark = new CANSparkMax(PORTS.ROTATE_SPARK, MotorType.kBrushless);
		
		falconTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
		rotateEncoder = rotateSpark.getEncoder();
		// Show encoder data in degrees of rotation (with 0 being straight forward)
		rotateEncoder.setPositionConversionFactor((1.0/7.0) * (1.0/5.0) * (16.0/132.0) * 360.0); // 1:7 gear ratio times 1:5 times 16:132 sprockets

		rotateLimitLeft = new DigitalInput(PORTS.ROTATE_LIMIT_LEFT);
		rotateLimitCenter = new DigitalInput(PORTS.ROTATE_LIMIT_CENTER);
		rotateLimitRight = new DigitalInput(PORTS.ROTATE_LIMIT_RIGHT);
		
		rotatePID = new PIDController(ROTATE_Kp, ROTATE_Ki, ROTATE_Kd);
		falconPID = new PIDController(FALCON_Kp, FALCON_Ki, FALCON_Kd);

		this.lime = Robot.lime;

		turretInit();
	}

	private void turretInit() {
		// Factory Default all hardware to prevent unexpected behaviour
		falconTalon.configFactoryDefault();

		// Set Neutral Mode to Brake so Back-EMF doesn't break the robot
		falconTalon.setNeutralMode(NeutralMode.Brake);
	}

	public void periodic() {
		shoot();
		rotate();
	}

	/**
	 * Set the shooter speed based on controller input
	 * @param controller
	 */
	private void shoot() {
		setShooterSpeed();
		rampSpeed();

		double falconOutputSpeed = shooterSpeed;
		if(usePID) {

			falconOutputSpeed = falconPID.calculate(
				-((double) falconTalon.getSelectedSensorVelocity())/MAX_FALCON_ENCODER_RATE,
				shooterSpeed
			);
			if(shooterSpeed == 0) {
				falconOutputSpeed = 0;
			}
			else if(feedForward) {
				falconOutputSpeed += shooterSpeed;
			}
		}

		System.out.println("Falcon Speed: " + -falconOutputSpeed);
		SmartDashboard.putNumber("Falcon Output Speed", falconOutputSpeed);
		falconTalon.set(ControlMode.PercentOutput, -falconOutputSpeed);

		SmartDashboard.putNumber("shooter speed", shooterSpeed);
		SmartDashboard.putNumber("ramp target shooter speed", targetSpeed);

		double falconRate = ((double) falconTalon.getSelectedSensorVelocity())*(6_500.0/20_000.0);
		SmartDashboard.putNumber("falcon encoder rate", falconRate);
		SmartDashboard.putNumber("falcon current", falconTalon.getSupplyCurrent());
		SmartDashboard.putNumber("rotate encoder", rotateEncoder.getPosition());

		SmartDashboard.putString("ramp state", rampState.toString());

		SmartDashboard.putBoolean("rotate limit left", rotateLimitLeft.get());
		SmartDashboard.putBoolean("rotate limit center", rotateLimitCenter.get());
		SmartDashboard.putBoolean("rotate limit right", rotateLimitRight.get());
	}

	private void setShooterSpeedAuton(){
		if(targetSpeed == 0) { // First time this is run in auton
			startSpeed = 0;
			targetSpeed = 0.7; // Best speed for target position
			rampState = RAMP_STATE.UP;
			setRampPeriod();
		}
	}
	private void setShooterSpeed() {
		// Speed -> 0
		if(driverstation.getRawButtonPressed(BUTTONS.DRIVER_STATION.COL_BUTTON_1)) {
			// Ramp shooter down
			startSpeed = shooterSpeed;
			targetSpeed = 0;
			rampState = RAMP_STATE.DOWN;
			setRampPeriod();
			System.out.println("Shooter Speed (ramp) -> 0%");
			System.out.println("Intake Speed -> 0%");
		}

		// Use manual control
		if(driverstation.getRawButton(BUTTONS.DRIVER_STATION.LEFT_TOGGLE_BUTTON)) {
			startSpeed = shooterSpeed;
			// Add one and divide by two to convert from -1:1 to 0:1
			targetSpeed = (driverstation.getRawAxis(BUTTONS.DRIVER_STATION.LEFT_SLIDER) + 1.0) / 2.0;
			if(targetSpeed > maxSpeed) // Don't want to try to go too fast
				targetSpeed = maxSpeed;
			else if(Math.abs(targetSpeed) <= 0.1) // Deadband
				targetSpeed = 0;
			
			if(shooterSpeed < targetSpeed)
				rampState = RAMP_STATE.UP;
			else
				rampState = RAMP_STATE.DOWN;
			setRampPeriod();
		}
		// Use auto control if limelight is enabled
		// Or max speed if it's disabled (does this automatically since distance reads as 9.5m when there's no target)
		else if(driverstation.getRawButtonPressed(BUTTONS.DRIVER_STATION.COL_BUTTON_2)) {
			startSpeed = shooterSpeed;
			targetSpeed = lime.getTargetShooterSpeed();
			if(targetSpeed > maxSpeed) // Don't want to try to go too fast
				targetSpeed = maxSpeed;
			
			if(shooterSpeed < targetSpeed)
				rampState = RAMP_STATE.UP;
			else
				rampState = RAMP_STATE.DOWN;
			setRampPeriod();
		}
	}

	private void rampSpeed() {
		// Ramp the speed
		if(rampState == RAMP_STATE.DOWN) {
			if(shooterSpeed > targetSpeed) // If we have started the ramp down
				shooterSpeed += 0.02 * (targetSpeed - startSpeed) / rampPeriod; // 20 is because this iterates every 20 ms
			else {
				rampState = RAMP_STATE.NO_RAMP;
				targetSpeed = 0;
				startSpeed = 0;
				// System.out.println("Target shooter speed achieved");
			}
		}
		else if(rampState == RAMP_STATE.UP) {
			if(shooterSpeed < targetSpeed) // If we have started the ramp up
				shooterSpeed += 0.02 * (targetSpeed - startSpeed) / rampPeriod; // 20 is because this iterates every 20 ms
			else {
				rampState = RAMP_STATE.NO_RAMP;
				targetSpeed = 0;
				startSpeed = 0;
				// System.out.println("Target shooter speed achieved");
			}
		}
	}

	/**
	 * Sets the period that the shooter will ramp up over
	 * based on the desired change in speed
	 */
	private void setRampPeriod() {
		// Make time to ramp 5x the change in speed
		// Ex: 0.1 (10%) -> 0.6 (60%) is a 0.5 (50%)
		// change, so it will take 1.25 seconds to ramp up
		double time = Math.abs(shooterSpeed - targetSpeed)*2.5;
		// Make ramp down slower than ramp up to protect motor
		if(rampState == RAMP_STATE.DOWN)
			time *= 1.5;

		rampPeriod = time;
	}

	/**
	 * Sets the rotation speed of the turret
	 * @param controller
	 */
	private void rotate() {
		/*if(!turretZeroed) { // Don't move if not zeroed yet
			zeroTurret();
			return;
		}*/

		// If limit is clicked, zero the encoder
		// If ONLY home limit is clicked
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

		boolean clockwise = driverstation.getRawButton(BUTTONS.DRIVER_STATION.ROCKER_L_UP);
		boolean counterclockwise = driverstation.getRawButton(BUTTONS.DRIVER_STATION.ROCKER_L_DOWN);

		double speed = 0;
		double turnRate = 0.1;
		if(counterclockwise) { // Counterclockwise
			// if(rotateEncoder.getPosition() < TURRET_ANGLE_COUNTERCLOCKWISE_MAX) {
			if(!getRotateLimit(Direction.COUNTERCLOCKWISE)) {
				speed = turnRate;
			}
		}
		else if(clockwise) { //Clockwise
			if(!getRotateLimit(Direction.CLOCKWISE)) { // If limit is not activated
				speed = -turnRate;
			}
		}
		else if(lime.isTracking() && autoRotate) {
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
		double rotateSpeedMult = 1.0;
		double rotateEncVal = rotateEncoder.getPosition();
		if((rotateEncVal > 0 && Math.abs(TURRET_ANGLE_COUNTERCLOCKWISE_MAX - rotateEncVal) < 20) || 
			(rotateEncVal < 0 && Math.abs(TURRET_ANGLE_CLOCKWISE_MAX - rotateEncVal) < 20))
			rotateSpeedMult = 0.5;
		rotateSpark.set(speed*rotateSpeedMult);
		//System.out.println("Speed: " + speed*rotateSpeedMult);

	}

	private void zeroTurret() {
		double rotateSpeed = 0;
		turretZeroed = true;

        if(getHomeLimit()) { // If home limit is clicked
			rotateEncoder.setPosition(0);
			System.out.println("Turret Zeroed");
        }
        else if(getRotateLimit(Direction.COUNTERCLOCKWISE)) { // If max extension limit is clicked
			rotateEncoder.setPosition(TURRET_ANGLE_COUNTERCLOCKWISE_MAX);
			System.out.println("Turret Zeroed");
        }
        else if(getRotateLimit(Direction.CLOCKWISE)) { // If max extension limit is clicked
			rotateEncoder.setPosition(TURRET_ANGLE_CLOCKWISE_MAX);
			System.out.println("Turret Zeroed");
        }
        else {
			rotateSpeed = -0.07; // Slowly rotate counterclockwise ... until limit is hit
			turretZeroed = false;
		}
		rotateSpark.set(rotateSpeed);
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
			return !rotateLimitRight.get();
		// If rotating counterclockwise
		return !rotateLimitLeft.get();
	}
}