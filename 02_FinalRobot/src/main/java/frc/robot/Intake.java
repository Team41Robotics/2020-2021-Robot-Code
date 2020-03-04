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

	private Timer resetTimer;
	private Timer bottomLimTimer, middleLimTimer;
	private enum INTAKE_STATE {ZERO, PRE_ONE, ONE, TWO_UP, TWO_FALL, TWO_DOWN, THREE};
	private INTAKE_STATE intakeState = INTAKE_STATE.ZERO;

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

		resetTimer = new Timer();
		bottomLimTimer = new Timer();
		middleLimTimer = new Timer();

		intakeInit();
	}

	private void intakeInit() {
		shooterIntakeTalon.configFactoryDefault();
		shooterIntakeTalon.setNeutralMode(NeutralMode.Brake);

		intakeArmForward = true;
		intakeArmSol.set(Value.kForward);
	}

	public void controllerMove() {
		maxElevatorSpeedTop = SmartDashboard.getNumber("max elevator speed top", maxElevatorSpeedTop);
		maxElevatorSpeedBottom = SmartDashboard.getNumber("max elevator speed bottom", maxElevatorSpeedBottom);
		
		// boolean autoMode = SmartDashboard.getBoolean("autoMode", false);
		boolean autoMode = driverstation.getRawAxis(BUTTONS.DRIVER_STATION.LEFT_DIAL) > 0;
		if (autoMode) autoMode = autoIntake();
		if(!autoMode) {
			setIndexerSpeed();
		}
		setIntakeArmSpeed();
		toggleIntakeArm();

		SmartDashboard.putNumber("shooter intake speed", shooterIntakeSpeed);
		SmartDashboard.putNumber("elevator speed top", elevatorSpeedTop);
		SmartDashboard.putNumber("elevator speed bottom", elevatorSpeedBottom);
		SmartDashboard.putNumber("intake arm speed", intakeArmSpeed);

		SmartDashboard.putBoolean("elevator limit top", elevatorLimitTop.get());
		SmartDashboard.putBoolean("elevator limit middle", elevatorLimitMiddle.get());
		SmartDashboard.putBoolean("elevator limit bottom", elevatorLimitBottom.get());

		SmartDashboard.putString("intake state", intakeState.toString());
		SmartDashboard.putBoolean("autoMode", autoMode);
	}

	private boolean autoIntake() {
		boolean limTop = elevatorLimitTop.get();
		boolean limMid = elevatorLimitMiddle.get();
		boolean limBot = elevatorLimitBottom.get();
		/* if((intakeState != INTAKE_STATE.ONE) && !limTop && !limMid && !limBot) {
			// If we don't have any balls, start timer to reset ballCount
			// Don't do this if ball count is one since no balls will be hitting limits when that is the case
			if(resetTimer.get() == 0)
				resetTimer.start();
			else if(resetTimer.get() > 0.5) { // If we go half a second with no activations, we're sure we have no balls
				intakeState = INTAKE_STATE.ZERO;
				resetTimer.stop();
				resetTimer.reset();
			}
		}
		else if(resetTimer.get() != 0) { // If a limit got activated, reset the timer
			resetTimer.stop();
			resetTimer.reset();
		} */

		double speedTop = 0, speedBottom = 0;
		switch(intakeState) {
			case ZERO: // No balls yet
				// Run elevator until limit is pressed and unpressed, then stop top elevator
				speedTop = maxElevatorSpeedTop;
				speedBottom = maxElevatorSpeedBottom;
				if(limTop)
					intakeState = INTAKE_STATE.PRE_ONE;
				
				break;
			case PRE_ONE:
				// Make it go slower if the limit has been clicked so we don't overshoot
				speedTop = 0.5 * maxElevatorSpeedTop;
				speedBottom = maxElevatorSpeedBottom;
				if(!limTop) {
					speedTop = 0;
					intakeState = INTAKE_STATE.ONE;
				}
				
				break;
			case ONE:
				// Run bottom elevator up and down as long as only middle OR bottom limit is activated
				speedTop = 0;
				speedBottom = maxElevatorSpeedBottom;

				if(limMid && limBot) { // If we have three balls, both limits are pressed
					speedBottom = 0;
					intakeState = INTAKE_STATE.THREE;
				}
				else if(limMid) { // If we hit the top, go down
					intakeState = INTAKE_STATE.TWO_UP;
					middleLimTimer.reset();
					middleLimTimer.start();
				}
				break;
			case TWO_UP:
				speedTop = 0;
				speedBottom = maxElevatorSpeedBottom;
				if(limMid && limBot) { // If we have three balls, both limits are pressed
					speedBottom = 0;
					middleLimTimer.stop();
					middleLimTimer.reset();
					intakeState = INTAKE_STATE.THREE;
				}
				else if(middleLimTimer.get() > 0.5) {
					intakeState = INTAKE_STATE.TWO_FALL;
					middleLimTimer.stop();
					middleLimTimer.reset();
				}
				break;
			case TWO_FALL: // Make the second ball go down
				speedTop = 0;
				speedBottom = -maxElevatorSpeedBottom;
				if(limBot) {
					intakeState = INTAKE_STATE.TWO_DOWN;
					bottomLimTimer.reset();
					bottomLimTimer.start();
				}
				break;
			case TWO_DOWN: // Stop the second ball at the bottom for a second
				speedTop = 0;
				speedBottom = 0;
				if(bottomLimTimer.get() > 1) {
					bottomLimTimer.stop();
					bottomLimTimer.reset();
					intakeState = INTAKE_STATE.ONE;
				}
				break;
			case THREE:
				speedTop = 0;
				speedBottom = 0;
				elevatorSparkTop.set(speedTop);
				elevatorSparkBottom.set(speedBottom);
				return false;
		}
		elevatorSparkTop.set(speedTop);
		elevatorSparkBottom.set(speedBottom);
		return true;
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
			intakeArmSol.set(intakeArmForward ? Value.kReverse : Value.kForward);
			intakeArmForward = !intakeArmForward;
			System.out.println("Intake Arm -> " + (intakeArmForward ? "Forward" : "Reverse"));
		}
	}
}