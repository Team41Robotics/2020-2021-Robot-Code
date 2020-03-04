package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

class Climbing {
	private final MotorType kBrushless = MotorType.kBrushless;

	private Joystick driverstation, rightJoy;
	private CANSparkMax spark1, spark2;
	// private DoubleSolenoid climbSol;

	public Climbing() {
		driverstation = Robot.driverstation;
		rightJoy = Robot.rightJoy;
		spark1 = new CANSparkMax(PORTS.CLIMB_SPARK_1, kBrushless);
		spark2 = new CANSparkMax(PORTS.CLIMB_SPARK_2, kBrushless);
		// climbSol = new DoubleSolenoid(PORTS.CLIMB_SOL_FORWARD, PORTS.CLIMB_SOL_REVERSE);
	}

	public void controllerMove() {
		double speed = 0;
		if(driverstation.getRawButton(BUTTONS.DRIVER_STATION.RIGHT_TOGGLE_BUTTON)) {
			if(rightJoy.getPOV() == 0)
				speed = -0.75;
			if(rightJoy.getPOV() == 180)
				speed = 0.75;
			
			if(driverstation.getRawButtonPressed(BUTTONS.DRIVER_STATION.COL_BUTTON_3)) {
				// climbSol.set(climbSol.get() == Value.kForward ? Value.kReverse : Value.kForward);
			}
		}

		spark1.set(speed);
		spark2.set(speed);
	}
}