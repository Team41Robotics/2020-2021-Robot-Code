package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Turret {
	private TalonSRX shooterTalon1, shooterTalon2, intakeTalon;
	private CANSparkMax rotateSpark;
	private Encoder shooterEnc;
	private double shooterSpeed, intakeSpeed;

	public Turret() {
		shooterTalon1 = new TalonSRX(PORTS.SHOOTER_TALON_1);
		shooterTalon2 = new TalonSRX(PORTS.SHOOTER_TALON_2);
		intakeTalon = new TalonSRX(PORTS.INTAKE_TALON);
		rotateSpark = new CANSparkMax(PORTS.ROTATE_SPARK, MotorType.kBrushless);

		shooterEnc = new Encoder(PORTS.SHOOTER_ENCODER_A, PORTS.SHOOTER_ENCODER_B);
		shooterEnc.setDistancePerPulse(1.0/2048.0 * 60); // Shows encoder data in rpm

		shooterSpeed = 0.0;
		intakeSpeed = 0.0;

		shooterTalon1.setNeutralMode(NeutralMode.Brake);
		shooterTalon2.setNeutralMode(NeutralMode.Brake);
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
		//Change shooterSpeed using the x and y buttons
		//Y button increases shooterSpeed in the counterclockwise direction
		if(controller.getRawButtonPressed(BUTTONS.Y_BUTTON) && shooterSpeed <= 1.0) {
			shooterSpeed += 0.1;
			System.out.println("shooterSpeed: " + shooterSpeed);
		}
		//X button decreases shooterSpeed towards the clockwise direction
		if(controller.getRawButtonPressed(BUTTONS.X_BUTTON) && shooterSpeed >= -1.0) {
			shooterSpeed -= 0.1;
			System.out.println("shooterSpeed: " + shooterSpeed);
		}

		//Change intakeSpeed using the A and B buttons
		//B button increases shooterSpeed in the counterclockwise direction
		if(controller.getRawButtonPressed(BUTTONS.B_BUTTON) && intakeSpeed <= 1.0) {
			intakeSpeed += 0.1;
			System.out.println("intakeSpeed: " + intakeSpeed);
		}
		//A button decreases shooterSpeed towards the clockwise direction
		if(controller.getRawButtonPressed(BUTTONS.A_BUTTON) && intakeSpeed >= -1.0) {
			intakeSpeed -= 0.1;
			System.out.println("intakeSpeed: " + intakeSpeed);
		}

		//Stop the motors in the shooter using the left bumper
		if(controller.getRawButtonPressed(BUTTONS.L_BUMPER)){
			shooterSpeed = 0;
			intakeSpeed = 0;
		}

		shooterTalon1.set(ControlMode.PercentOutput, shooterSpeed); 
		shooterTalon2.set(ControlMode.PercentOutput, shooterSpeed);
		intakeTalon.set(ControlMode.PercentOutput, intakeSpeed);

		SmartDashboard.putNumber("shooter encoder rate", shooterEnc.getRate());
		SmartDashboard.putNumber("shooter talon 1 current", shooterTalon1.getStatorCurrent());
		SmartDashboard.putNumber("shooter talon 2 current", shooterTalon2.getStatorCurrent());
		
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
