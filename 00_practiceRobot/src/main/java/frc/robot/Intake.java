package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

class Intake {
	private final MotorType kBrushless = MotorType.kBrushless;

	private Joystick driverstation, leftJoy, rightJoy;
	private TalonSRX shooterIntakeTalon;
	private CANSparkMax elevatorSparkTop, elevatorSparkBottom, intakeArmSpark;
	private DoubleSolenoid intakeArmSol;

	private DigitalInput elevatorLimitTop, elevatorLimitMiddle, elevatorLimitBottom;

	private double shooterIntakeSpeed = 0, elevatorSpeedTop = 0, elevatorSpeedBottom = 0, intakeArmSpeed = 0;
	private boolean intakeArmForward;

	private Timer bottomLimTimer, middleLimTimer, topLimTimer;
	private enum RESET_STATE {NO_RESET, START, WAIT, END};
	private RESET_STATE resetState = RESET_STATE.NO_RESET;
	private enum INTAKE_STATE {ZERO, PRE_ONE, ONE, TWO_UP, TWO_FALL, TWO_DOWN, MANUAL};
	private INTAKE_STATE intakeState = INTAKE_STATE.ZERO;

	public enum AUTON_STATE {INIT, LOAD_BALLS, SHOOT};
	private AUTON_STATE autonState = AUTON_STATE.INIT;

	private double maxShooterIntakeSpeed = 0.8;
	private double maxElevatorSpeedTop = 0.5;
	private double maxElevatorSpeedBottom = 0.35;

	public Intake() {
		driverstation = Robot.driverstation;
		leftJoy = Robot.leftJoy;
		rightJoy = Robot.rightJoy;

		shooterIntakeTalon = new TalonSRX(PORTS.INTAKE_TALON);
		elevatorSparkTop = new CANSparkMax(PORTS.ELEVATOR_SPARK_TOP, kBrushless);
		elevatorSparkBottom = new CANSparkMax(PORTS.ELEVATOR_SPARK_BOTTOM, kBrushless);
		intakeArmSpark = new CANSparkMax(PORTS.BALL_INTAKE_SPARK, kBrushless);
		intakeArmSol = new DoubleSolenoid(PORTS.PCM, PORTS.INTAKE_SOL_FORWARD, PORTS.INTAKE_SOL_REVERSE);

		elevatorLimitTop = new DigitalInput(PORTS.ELEVATOR_LIMIT_TOP);
		elevatorLimitMiddle = new DigitalInput(PORTS.ELEVATOR_LIMIT_MIDDLE);
		elevatorLimitBottom = new DigitalInput(PORTS.ELEVATOR_LIMIT_BOTTOM);

		bottomLimTimer = new Timer();
		middleLimTimer = new Timer();
		topLimTimer = new Timer();

		intakeInit();
	}

	private void intakeInit() {
		shooterIntakeTalon.configFactoryDefault();
		shooterIntakeTalon.setNeutralMode(NeutralMode.Brake);

		intakeArmForward = false;
		intakeArmSol.set(Value.kReverse);
	}

	public void periodic() {
		if(Robot.inAuton) {
			auton();
		}
		else {
			int pov = driverstation.getPOV(0);
			if(pov != -1) pov /= 45; // Convert from 0-315 to 0-7
			boolean autoMode = (pov & 0b010) == 0b010; // If the second bit is active, use auto
			// Don't use auto mode if we will be climbing
			if(pov == -1)
				autoMode = false;
			// If we're not using auto mode, reset the state
			if(!autoMode) {
				resetState = RESET_STATE.NO_RESET;
				intakeState = INTAKE_STATE.ZERO;
			}

			if (autoMode) autoMode = autoIntake();
			if(!autoMode) { // If we're not using auto mode or we're in the manual stage
				setIndexerSpeed();
			}
			resetAutoIntake();
			setShooterIntakeSpeed();
			setIntakeArmSpeed();
			toggleIntakeArm();

			SmartDashboard.putBoolean("autoMode", autoMode);
		}

		maxElevatorSpeedTop = SmartDashboard.getNumber("max elevator speed top", maxElevatorSpeedTop);
		maxElevatorSpeedBottom = SmartDashboard.getNumber("max elevator speed bottom", maxElevatorSpeedBottom);

		SmartDashboard.putNumber("shooter intake speed", shooterIntakeSpeed);
		SmartDashboard.putNumber("elevator speed top", elevatorSpeedTop);
		SmartDashboard.putNumber("elevator speed bottom", elevatorSpeedBottom);
		SmartDashboard.putNumber("intake arm speed", intakeArmSpeed);

		SmartDashboard.putBoolean("elevator limit top", elevatorLimitTop.get());
		SmartDashboard.putBoolean("elevator limit middle", elevatorLimitMiddle.get());
		SmartDashboard.putBoolean("elevator limit bottom", elevatorLimitBottom.get());

		SmartDashboard.putString("intake state", intakeState.toString());
		SmartDashboard.putString("intake auton state", autonState.toString());
	}

	private void auton() {
		switch(autonState) {
			case INIT:
				// Put the intake into manual control since we will start with three balls
				intakeState = INTAKE_STATE.MANUAL;
				autonState = AUTON_STATE.LOAD_BALLS;
				break;
			case LOAD_BALLS:
				autoIntake();
				break;
			case SHOOT:
				// If turret is up to speed, then shoot
				shooterIntakeSpeed = maxShooterIntakeSpeed;
				elevatorSpeedTop = maxElevatorSpeedTop;
				elevatorSpeedBottom = maxElevatorSpeedBottom;

				shooterIntakeTalon.set(ControlMode.PercentOutput, -maxShooterIntakeSpeed);
				elevatorSparkTop.set(elevatorSpeedTop);
				elevatorSparkBottom.set(elevatorSpeedBottom);

				// resetAutoIntake();
				// If we shot balls and the intake reset itslef
				/* if(intakeState == INTAKE_STATE.ZERO) {		
					autonState = AUTON_STATE.LOAD_BALLS;
					shooterIntakeTalon.set(ControlMode.PercentOutput, 0);
					elevatorSparkTop.set(0);
					elevatorSparkBottom.set(0);
				} */
				break;
		}
	}

	private boolean autoIntake() {
		boolean limTop = elevatorLimitTop.get();
		boolean limMid = elevatorLimitMiddle.get();
		boolean limBot = elevatorLimitBottom.get();

		switch(intakeState) {
			case ZERO: // No balls yet
				// Run elevator until limit is pressed and unpressed, then stop top elevator
				elevatorSpeedTop = maxElevatorSpeedTop;
				elevatorSpeedBottom = maxElevatorSpeedBottom;
				// if(limTop && limMid) // If both are activated, we have two balls, so move past PRE_ONE to avoid squishing
				// 	intakeState = INTAKE_STATE.ONE;
				if(limTop)
					intakeState = INTAKE_STATE.PRE_ONE;
				break;
			case PRE_ONE:
				// Make it go slower if the limit has been clicked so we don't overshoot
				elevatorSpeedTop = 0.5 * maxElevatorSpeedTop;
				elevatorSpeedBottom = maxElevatorSpeedBottom;
				if(!limTop) {
					elevatorSpeedTop = 0;
					intakeState = INTAKE_STATE.ONE;
				}
				break;
			case ONE:
				// Run bottom elevator up and down as long as only middle OR bottom limit is activated
				elevatorSpeedTop = 0;
				elevatorSpeedBottom = maxElevatorSpeedBottom;

				if(limMid && limBot) { // If we have three balls, both limits are pressed
					elevatorSpeedBottom = 0;
					intakeState = INTAKE_STATE.MANUAL;
				}
				else if(limMid) { // If we hit the top, go down
					intakeState = INTAKE_STATE.TWO_UP;
					middleLimTimer.reset();
					middleLimTimer.start();
				}
				break;
			case TWO_UP:
				elevatorSpeedTop = 0;
				elevatorSpeedBottom = maxElevatorSpeedBottom;
				if(limMid && limBot) { // If we have three balls, both limits are pressed
					elevatorSpeedBottom = 0;
					middleLimTimer.stop();
					middleLimTimer.reset();
					intakeState = INTAKE_STATE.MANUAL;
				}
				else if(middleLimTimer.get() > 0.4) {
					intakeState = INTAKE_STATE.TWO_FALL;
					middleLimTimer.stop();
					middleLimTimer.reset();
				}
				break;
			case TWO_FALL: // Make the second ball go down
				elevatorSpeedTop = 0;
				elevatorSpeedBottom = -maxElevatorSpeedBottom;
				if(limBot) {
					intakeState = INTAKE_STATE.TWO_DOWN;
					bottomLimTimer.reset();
					bottomLimTimer.start();
				}
				break;
			case TWO_DOWN: // Stop the second ball at the bottom for a second
				elevatorSpeedTop = 0;
				elevatorSpeedBottom = 0;
				if(bottomLimTimer.get() > 1) {
					bottomLimTimer.stop();
					bottomLimTimer.reset();
					intakeState = INTAKE_STATE.ONE;
				}
				break;
			case MANUAL:
				elevatorSpeedTop = 0;
				elevatorSpeedBottom = 0;
				elevatorSparkTop.set(elevatorSpeedTop);
				elevatorSparkBottom.set(elevatorSpeedBottom);
				return false;
		}
		elevatorSparkTop.set(elevatorSpeedTop);
		elevatorSparkBottom.set(elevatorSpeedBottom);
		return true;
	}

	private void resetAutoIntake() {
		boolean limTop = elevatorLimitTop.get();

		switch(resetState) {
			case NO_RESET:
				// If we're not in state zero but are running all the motors
				// then reset, because that means we were shooting
				if(intakeState != INTAKE_STATE.ZERO && shooterIntakeSpeed != 0 && elevatorSpeedTop != 0 && elevatorSpeedBottom != 0)
					resetState = RESET_STATE.START;
				break;
			case START:
				topLimTimer.start();
				resetState = RESET_STATE.WAIT;
				break;
			case WAIT:
				if(limTop) {
					topLimTimer.stop();
					topLimTimer.reset();

					resetState = RESET_STATE.START;
				}
				else if(topLimTimer.get() > 3) // If we go three seconds without shooting, reset
					resetState = RESET_STATE.END;
				break;
			case END:
				intakeState = INTAKE_STATE.ZERO;
				resetState = RESET_STATE.NO_RESET;
				break;
		}
	}

	private void setIndexerSpeed() {
		// Shut off button
		if(driverstation.getRawButtonPressed(BUTTONS.DRIVER_STATION.COL_BUTTON_1)) {
			shooterIntakeSpeed = 0;
			elevatorSpeedTop = 0;
			elevatorSpeedBottom = 0;

			System.out.println("Shooter Intake Speed -> 0%");
			System.out.println("Elevator Speed Top -> 0%");
			System.out.println("Elevator Speed Bottom -> 0%");
		}
		
		// Elevator Top
		if(driverstation.getRawButton(BUTTONS.DRIVER_STATION.ROCKER_M_UP)) {
			if(elevatorSpeedTop == 0) // Only print the first time it's changed
				System.out.println("Elevator Speed Top -> " + Math.round(elevatorSpeedTop*100.0) + "%");
			elevatorSpeedTop = maxElevatorSpeedTop;
		}
		else if(driverstation.getRawButton(BUTTONS.DRIVER_STATION.ROCKER_M_DOWN) && elevatorSpeedTop >= 0) {
			if(elevatorSpeedTop == 0) // Only print the first time it's changed
				System.out.println("Elevator Speed Top -> " + Math.round(elevatorSpeedTop*100.0) + "%");
			elevatorSpeedTop = -maxElevatorSpeedTop;
		}
		else {
			if(elevatorSpeedTop != 0) // Only print the first time it's changed
				System.out.println("Elevator Speed Top -> " + Math.round(elevatorSpeedTop*100.0) + "%");
			elevatorSpeedTop = 0;
		}
		elevatorSparkTop.set(elevatorSpeedTop);
		
		// Elevator Bottom
		if(driverstation.getRawButton(BUTTONS.DRIVER_STATION.ROCKER_R_UP)) {
			if(elevatorSpeedBottom == 0) // Only print the first time it's changed
				System.out.println("Elevator Speed Bottom -> " + Math.round(elevatorSpeedBottom*100.0) + "%");
			elevatorSpeedBottom = maxElevatorSpeedBottom;
		}
		else if(driverstation.getRawButton(BUTTONS.DRIVER_STATION.ROCKER_R_DOWN) && elevatorSpeedBottom >= 0) {
			if(elevatorSpeedBottom == 0) // Only print the first time it's changed
				System.out.println("Elevator Speed Bottom -> " + Math.round(elevatorSpeedBottom*100.0) + "%");
			elevatorSpeedBottom = -maxElevatorSpeedBottom;
		}
		else {
			if(elevatorSpeedBottom != 0) // Only print the first time it's changed
				System.out.println("Elevator Speed Bottom -> " + Math.round(elevatorSpeedBottom*100.0) + "%");
			elevatorSpeedBottom = 0;
		}
		elevatorSparkBottom.set(elevatorSpeedBottom);
	}
	private void setShooterIntakeSpeed() {
		// Shooter Intake
		if(driverstation.getRawButton(BUTTONS.DRIVER_STATION.TOGGLE_SWITCH_L)) {
			if(shooterIntakeSpeed == 0) // Only print the first time it's changed
				System.out.println("Shooter Intake Speed -> 80%");
			shooterIntakeSpeed = maxShooterIntakeSpeed;
		}
		else {
			if(shooterIntakeSpeed != 0) // Only print the first time it's changed
				System.out.println("Shooter Intake Speed -> 0%");
			shooterIntakeSpeed = 0;
		}
		shooterIntakeTalon.set(ControlMode.PercentOutput, -shooterIntakeSpeed);
	}
	private void setIntakeArmSpeed() {
		if(rightJoy.getRawButton(BUTTONS.DRIVER_STATION.R_JOY_TRIGGER)) {
			if(intakeArmSpeed == 0) // Only print the first time it's changed
				System.out.println("Intake Arm Speed -> " + Math.round(intakeArmSpeed*100.0) + "%");
			intakeArmSpeed = 0.7;
		}
		else {
			if(intakeArmSpeed != 0) // Only print the first time it's changed
				System.out.println("Intake Arm Speed -> " + Math.round(intakeArmSpeed*100.0) + "%");
			intakeArmSpeed = 0;
		}
		intakeArmSpark.set(intakeArmSpeed);
	}
	
	private void toggleIntakeArm() {
		if(leftJoy.getRawButtonPressed(BUTTONS.DRIVER_STATION.L_JOY_TRIGGER)) {
			setIntakeArmPos(!intakeArmForward);
		}
	}

	public void setIntakeArmPos(boolean forward) {
		intakeArmSol.set(forward ? Value.kForward : Value.kReverse);
		intakeArmForward = forward;
		System.out.println("Intake Arm -> " + (intakeArmForward ? "Forward" : "Reverse"));
	}

	public void setAutonState(AUTON_STATE state) {
		autonState = state;
	}
}