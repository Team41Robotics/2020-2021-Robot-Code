package frc.robot;

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
	private double shooterSpeed, intakeSpeed;
	private PIDController falconPID, rotatePID;
	private Vision vis;

	private boolean rampUp = false;
	private boolean rampDown = false;
	private boolean turretZeroed = false;
	
	private final double maxSpeed = 0.7; // From 0 to 1
	private final double rampUpPeriod = 3.0; // In seconds. If this is 0, it will immediately go to maxSpeed
	private final double rampDownPeriod = 7.0; // In seconds. If this is 0, it will immediately go to maxSpeed
	private final boolean feedForward = true;
	private final boolean usePID = true;
	private final boolean useVision = true;
	
	// Constants
	private final double MAX_FALCON_ENCODER_RATE = 6_500; // Found experimentally
	private final double TURRET_ENC_MAX = 140; // Found experimentally
	private final double FALCON_Kp = 1.2, FALCON_Ki = 0.2, FALCON_Kd = 0.0;
	private final double ROTATE_Kp = 0.012, ROTATE_Ki = 0.004, ROTATE_Kd = 0.0; //.012, .004, 0

	public Turret(Vision vis) {
		falconTalon = new TalonFX(PORTS.FALCON_TALON);
		intakeTalon = new TalonSRX(PORTS.INTAKE_TALON);
		rotateSpark = new CANSparkMax(PORTS.ROTATE_SPARK, MotorType.kBrushless);
		
		// falconEncoder = new CANCoder(PORTS.FALCON_TALON);
		// falconEncoder.setDistancePerPulse(1.0/2048.0 * 60); // Shows encoder data in gearRatio*rpm
		axleEncoder = new Encoder(PORTS.AXLE_ENCODER_A, PORTS.AXLE_ENCODER_B);
		axleEncoder.setDistancePerPulse(1.0/2048.0 * 60); // Shows encoder data in rpm
		rotateEncoder = rotateSpark.getEncoder();

		rotateLimit = new DigitalInput(PORTS.ROTATE_LIMIT);
		
		rotatePID = new PIDController(ROTATE_Kp, ROTATE_Ki, ROTATE_Kd);
		falconPID = new PIDController(FALCON_Kp, FALCON_Ki, FALCON_Kd);

		this.vis = vis;

		turretInit();
	}

	private void turretInit() {
		shooterSpeed = 0.0;
		intakeSpeed = 0.0;

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

		//Stop the motors in the shooter using the left bumper
		if(controller.getRawButtonPressed(BUTTONS.L_BUMPER)) {
			intakeSpeed = 0;
			rampUp = false;
			if(rampDownPeriod == 0)
				shooterSpeed = 0;
			else
				rampDown = true;
		}

		if(usePID) {
			double falconOutputSpeed = falconPID.calculate(
				axleEncoder.getRate()/MAX_FALCON_ENCODER_RATE,
				shooterSpeed
			);
			if(shooterSpeed == 0) {
				falconOutputSpeed = 0;
			}
			else if(feedForward) {
				falconOutputSpeed += shooterSpeed;
			}

			falconTalon.set(ControlMode.PercentOutput, falconOutputSpeed);
		}
		else {
			falconTalon.set(ControlMode.PercentOutput, shooterSpeed);
		}
		intakeTalon.set(ControlMode.PercentOutput, -intakeSpeed);

		SmartDashboard.putNumber("shooter encoder rate", axleEncoder.getRate());
		SmartDashboard.putNumber("falcon current", falconTalon.getSupplyCurrent());
		// SmartDashboard.putNumber("falcon encoder rate", falconEncoder.getVelocity());
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

		if(controller.getRawButtonPressed(BUTTONS.R_BUMPER)) { // Start ramping up the speed
			if(rampUpPeriod == 0) {
				shooterSpeed = maxSpeed;
				System.out.println("Starting ramp up to " + Math.round(maxSpeed*100) + "% speed immediately");
			}
			else {
				rampUp = true;
				System.out.println("Starting ramp up to " + Math.round(maxSpeed*100) + "% speed over " + rampUpPeriod + " seconds");
			}
		}
		if(rampDown) {
			if(shooterSpeed > 0) // If we have started the ramp down
				shooterSpeed -= 20/(rampDownPeriod*1000) * maxSpeed; // 20 is because this iterates every 20 ms
			else
				rampDown = false;
		}
		if(rampUp) {
			if(shooterSpeed < maxSpeed) // If we have started the ramp up
				shooterSpeed += 20/(rampUpPeriod*1000) * maxSpeed; // 20 is because this iterates every 20 ms
			else
				rampUp = false;
		}
	}
	private void setIntakeSpeed(Joystick controller) {
		//Change intakeSpeed using the A and B buttons
		//B button increases shooterSpeed in the counterclockwise direction
		if(controller.getRawButtonPressed(BUTTONS.B_BUTTON) && intakeSpeed <= 1.0) {
			intakeSpeed += 0.2;
			System.out.println("intakeSpeed: " + Math.round(intakeSpeed*100.0) + "%");
		}
		//A button decreases shooterSpeed towards the clockwise direction
		if(controller.getRawButtonPressed(BUTTONS.A_BUTTON) && intakeSpeed >= -1.0) {
			intakeSpeed -= 0.2;
			System.out.println("intakeSpeed: " + Math.round(intakeSpeed*100.0) + "%");
		}
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
			rotateEncoder.setPosition(0);
		}

		double leftTrigger = controller.getRawAxis(BUTTONS.LEFT_TRIGGER_AXIS);
		double rightTrigger = controller.getRawAxis(BUTTONS.RIGHT_TRIGGER_AXIS);

		double speed = 0;
		double speedMultiplier = 0.20;
		if(leftTrigger > 0) { // Counterclockwise
			// if(rotateEncoder.getPosition() < TURRET_ENC_MAX) {
				speed = leftTrigger*leftTrigger*leftTrigger*speedMultiplier;
			// }
		}
		else if(rightTrigger > 0) { //Clockwise
			if(rotateLimit.get()) { // If limit is not activated
				speed = -rightTrigger*rightTrigger*rightTrigger*speedMultiplier;
			}
		}
		else if(vis.isTracking() && useVision) {
			double tolerance = .25; // In degrees
			double tx = vis.getTX();
			if(tx > tolerance) {
				// Since it's currently aimed to the left of the center
				// of the target, we want to adjust by moving clockwise
				if(rotateLimit.get()) {// Limit reads true when not clicked
					// speed = -0.1;
					speed = rotatePID.calculate(0, -tx);
				}
			}
			else if(tx < -tolerance) {
				// Since it's currently aimed to the right of the center
				// of the target, we want to adjust by moving clockwise
				if(rotateEncoder.getPosition() < TURRET_ENC_MAX) {
					// speed = 0.1;
					speed = rotatePID.calculate(0, -tx);
				}
			} else {
				rotatePID.reset(); //resets accumulated integral error when within threshold
			}

			SmartDashboard.putNumber("Rotation Speed", speed);
		}
		rotateSpark.set(speed);
	}

	private void zeroTurret() {
        if(!rotateLimit.get()) { // Limit reads false when clicked
			rotateEncoder.setPosition(0);
            turretZeroed = true;
			rotateSpark.set(0);
			System.out.println("Turret zeroed");
        }
        else{
			rotateSpark.set(-0.2); // Slowly rotate clockwise ... until limit is hit
		}
    }
}