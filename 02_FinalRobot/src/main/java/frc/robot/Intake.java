package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

class Intake {
	private final MotorType kBrushless = MotorType.kBrushless;

	private Joystick controller, extraJoy;
	private TalonSRX shooterIntakeTalon;
	private CANSparkMax elevatorSparkTop, elevatorSparkBottom, intakeArmSpark;
	private DoubleSolenoid intakeArmSol;

	private DigitalInput elevatorLimitTop, elevatorLimitMiddle, elevatorLimitBottom;

	private double shooterIntakeSpeed = 0, elevatorSpeedTop = 0, elevatorSpeedBottom = 0, intakeArmSpeed = 0;
	private boolean intakeArmForward;

	public Intake() {
		controller = Robot.controller;
		extraJoy = Robot.extraJoy;

		shooterIntakeTalon = new TalonSRX(PORTS.INTAKE_TALON);
		elevatorSparkTop = new CANSparkMax(PORTS.ELEVATOR_SPARK_TOP, kBrushless);
		elevatorSparkBottom = new CANSparkMax(PORTS.ELEVATOR_SPARK_BOTTOM, kBrushless);
		intakeArmSpark = new CANSparkMax(PORTS.BALL_INTAKE_SPARK, kBrushless);
		intakeArmSol = new DoubleSolenoid(PORTS.PCM, PORTS.INTAKE_SOL_FORWARD, PORTS.INTAKE_SOL_REVERSE);

		elevatorLimitTop = new DigitalInput(PORTS.ELEVATOR_LIMIT_TOP);
		elevatorLimitMiddle = new DigitalInput(PORTS.ELEVATOR_LIMIT_MIDDLE);
		elevatorLimitBottom = new DigitalInput(PORTS.ELEVATOR_LIMIT_BOTTOM);

		intakeInit();
	}

	private void intakeInit() {
		shooterIntakeTalon.configFactoryDefault();
		shooterIntakeTalon.setNeutralMode(NeutralMode.Brake);

		intakeArmForward = true;
		intakeArmSol.set(Value.kForward);
	}

	public void controllerMove() {
		setIndexerSpeed();
		setIntakeArmSpeed();
		toggleIntakeArm();

		SmartDashboard.putNumber("shooter intake speed", shooterIntakeSpeed);
		SmartDashboard.putNumber("elevator speed top", elevatorSpeedTop);
		SmartDashboard.putNumber("elevator speed bottom", elevatorSpeedBottom);
		SmartDashboard.putNumber("intake arm speed", intakeArmSpeed);
		
		SmartDashboard.putBoolean("elevator limit top", elevatorLimitTop.get());
		SmartDashboard.putBoolean("elevator limit middle", elevatorLimitMiddle.get());
		SmartDashboard.putBoolean("elevator limit bottom", elevatorLimitBottom.get());
	}

	private void setIndexerSpeed() {
		// Shut off button
		if(controller.getRawButtonPressed(BUTTONS.GAMEPAD.L_BUMPER)) {
			shooterIntakeSpeed = 0;
			elevatorSpeedTop = 0;
			elevatorSpeedBottom = 0;
		}
		
		// Shooter Intake
		double shooterIntakeSpeedIncrement = 0.8;
		if(extraJoy.getRawButtonPressed(BUTTONS.BIG_JOY.LEFT_BACK_BUTTON)) {
			if(shooterIntakeSpeed == shooterIntakeSpeedIncrement)
				shooterIntakeSpeed = 0;
			else
				shooterIntakeSpeed = shooterIntakeSpeedIncrement;
			System.out.println("Shooter Intake Speed -> " + Math.round(shooterIntakeSpeed*100.0) + "%");
		}
		shooterIntakeTalon.set(ControlMode.PercentOutput, -shooterIntakeSpeed);
		
		// Elevator Top
		double elevatorSpeedTopIncrement = 0.5;
		// B button increases shooterSpeed in the shooting direction
		if(controller.getRawButtonPressed(BUTTONS.GAMEPAD.B_BUTTON) && elevatorSpeedTop < 1.0) {
			elevatorSpeedTop += elevatorSpeedTopIncrement;
			System.out.println("Elevator Speed Top -> " + Math.round(elevatorSpeedTop*100.0) + "%");
		}
		// A button decreases shooterSpeed towards the reverse direction
		if(controller.getRawButtonPressed(BUTTONS.GAMEPAD.A_BUTTON) && elevatorSpeedTop >= 0) {
			elevatorSpeedTop -= elevatorSpeedTopIncrement;
			System.out.println("Elevator Speed Top -> " + Math.round(elevatorSpeedTop*100.0) + "%");
		}
		elevatorSparkTop.set(elevatorSpeedTop);

		// Elevator Bottom
		double elevatorSpeedBottomIncrement = 0.3;
		if(extraJoy.getRawButtonPressed(BUTTONS.BIG_JOY.RIGHT_FRONT_BUTTON)) {
			elevatorSpeedBottom += elevatorSpeedBottomIncrement;
			System.out.println("Elevator Speed Bottom -> " + Math.round(elevatorSpeedBottom*100.0) + "%");
		}
		if(extraJoy.getRawButtonPressed(BUTTONS.BIG_JOY.RIGHT_BACK_BUTTON)) {
			elevatorSpeedBottom -= elevatorSpeedBottomIncrement;
			System.out.println("Elevator Speed Bottom -> " + Math.round(elevatorSpeedBottom*100.0) + "%");
		}
		
		elevatorSparkBottom.set(elevatorSpeedBottom);
	}
	private void setIntakeArmSpeed() {
		double intakeArmSpeedIncrement = 0.7;
		if(extraJoy.getRawButtonPressed(BUTTONS.BIG_JOY.LEFT_FRONT_BUTTON)) {
			if(intakeArmSpeed == intakeArmSpeedIncrement)
				intakeArmSpeed = 0;
			else
				intakeArmSpeed = intakeArmSpeedIncrement;
			System.out.println("Intake Arm Speed -> " + Math.round(intakeArmSpeed*100.0) + "%");
		}
		intakeArmSpark.set(intakeArmSpeed);
	}
	
	private void toggleIntakeArm() {
		if(extraJoy.getRawButtonPressed(BUTTONS.BIG_JOY.TRIGGER)) {
			intakeArmSol.set(intakeArmForward ? Value.kReverse : Value.kForward);
			intakeArmForward = !intakeArmForward;
			System.out.println("Intake Arm -> " + (intakeArmForward ? "Forward" : "Reverse"));
		}
	}
}