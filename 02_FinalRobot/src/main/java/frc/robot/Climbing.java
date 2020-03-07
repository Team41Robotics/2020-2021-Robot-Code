package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

class Climbing {
	private final MotorType kBrushless = MotorType.kBrushless;

	private Joystick driverstation, rightJoy;
	private CANSparkMax spark1, spark2;
	private CANEncoder climbEnc;
	private DoubleSolenoid piston;

	private boolean useAuto = false;
	private boolean currentPovToggle = false;
	private Timer pistonTimer;
	private enum CLIMB_STATE {NONE, START_UP, START_UP_TRANS_CLAMP_DOWN, CLAMP_DOWN, CLAMP_DOWN_TRANS_PISTON, PISTON_FORWARD, PISTON_WAIT, PISTON_REVERSE, PISTON_TRANS_FORK_UP, FORK_UP, FORK_UP_TRANS_FINAL_UP, FINAL_UP};
	private CLIMB_STATE climbState = CLIMB_STATE.NONE;

	private final double CLIMB_POS_START_UP = 0000,
						CLIMB_POS_CLAMP_DOWN = 0000,
						CLIMB_POS_FORK_DROP = 0000,
						CLIMB_POS_FINAL = 0000;

	public Climbing() {
		driverstation = Robot.driverstation;
		rightJoy = Robot.rightJoy;
		spark1 = new CANSparkMax(PORTS.CLIMB_SPARK_1, kBrushless);
		spark2 = new CANSparkMax(PORTS.CLIMB_SPARK_2, kBrushless);
		climbEnc = spark1.getEncoder();
		piston = new DoubleSolenoid(PORTS.CLIMB_SOL_FORWARD, PORTS.CLIMB_SOL_REVERSE);
	}

	public void controllerMove() {
		double speed = 0;
		if(driverstation.getRawButton(BUTTONS.DRIVER_STATION.RIGHT_TOGGLE_BUTTON)) {
			if(rightJoy.getPOV() == 0)
				speed = -0.75;
			if(rightJoy.getPOV() == 180)
				speed = 0.75;
			
			if(driverstation.getRawButtonPressed(BUTTONS.DRIVER_STATION.COL_BUTTON_3)) {
				piston.set(piston.get() == Value.kForward ? Value.kReverse : Value.kForward);
			}
		}

		spark1.set(speed);
		spark2.set(speed);

		if(useAuto)
			useAuto = autoClimb();
	}

	private boolean autoClimb() {
		double climbSpeed = 0;
		int pov = driverstation.getPOV(0);
		if(pov != -1) pov /= 45; // Convert from degrees to positions from 0-7
		// Climbing uses a toggle of the third bit
		boolean povToggle = (pov & 0b100) == 0b100;
		switch(climbState) {
			case NONE:
				climbSpeed = 0;
				if(pov == 3)
					climbState = CLIMB_STATE.START_UP;
				break;
			case START_UP:
				climbSpeed = -0.5;
				if(climbEnc.getPosition() > CLIMB_POS_START_UP)
					climbState = CLIMB_STATE.START_UP_TRANS_CLAMP_DOWN;
				break;
			case START_UP_TRANS_CLAMP_DOWN:
				climbSpeed = 0;
				if(povToggle != currentPovToggle) {
					climbState = CLIMB_STATE.CLAMP_DOWN;
					currentPovToggle = povToggle;
				}
				break;
			case CLAMP_DOWN:
				climbSpeed = 0.5;
				if(climbEnc.getPosition() < CLIMB_POS_CLAMP_DOWN)
					climbState = CLIMB_STATE.CLAMP_DOWN_TRANS_PISTON;
				break;
			case CLAMP_DOWN_TRANS_PISTON:
				climbSpeed = 0;
				if(povToggle != currentPovToggle) {
					climbState = CLIMB_STATE.PISTON_FORWARD;
					currentPovToggle = povToggle;
				}
				break;
			case PISTON_FORWARD:
				climbSpeed = 0;
				piston.set(Value.kForward);
				pistonTimer.start();
				climbState = CLIMB_STATE.PISTON_WAIT;
				break;
			case PISTON_WAIT:
				climbSpeed = 0;
				if(pistonTimer.get() > 1) {
					pistonTimer.stop();
					climbState = CLIMB_STATE.PISTON_REVERSE;
				}
				break;
			case PISTON_REVERSE:
				climbSpeed = 0;
				piston.set(Value.kReverse);
				climbState = CLIMB_STATE.PISTON_TRANS_FORK_UP;
				break;
			case PISTON_TRANS_FORK_UP:
				climbSpeed = 0;
				if(povToggle != currentPovToggle) {
					climbState = CLIMB_STATE.FORK_UP;
					currentPovToggle = povToggle;
				}
				break;
			case FORK_UP: // Go until forks are dropped and robots can climb on
				climbSpeed = 0.5;
				if(climbEnc.getPosition() < CLIMB_POS_FORK_DROP)
					climbState = CLIMB_STATE.FORK_UP_TRANS_FINAL_UP;
				break;
			case FORK_UP_TRANS_FINAL_UP:
				climbSpeed = 0;
				if(povToggle != currentPovToggle) {
					climbState = CLIMB_STATE.FINAL_UP;
					currentPovToggle = povToggle;
				}
				break;
			case FINAL_UP:
				climbSpeed = 0.5;
				if(climbEnc.getPosition() < CLIMB_POS_FINAL)
					return false;
				break;
		}

		spark1.set(climbSpeed);
		spark2.set(climbSpeed);

		return true;
	}
}