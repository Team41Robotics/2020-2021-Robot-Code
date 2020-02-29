package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;

class Climbing {
	private final MotorType kBrushless = MotorType.kBrushless;

	private Joystick extraJoy;
	private CANSparkMax spark1, spark2;

	public Climbing() {
		extraJoy = Robot.extraJoy;
		spark1 = new CANSparkMax(PORTS.CLIMB_SPARK_1, kBrushless);
		spark2 = new CANSparkMax(PORTS.CLIMB_SPARK_2, kBrushless);
	}

	public void controllerMove() {
		// double speed = extraJoy.getRawAxis(BUTTONS.BIG_JOY.ROLLER);
		// speed = (speed + 1.0) / 2.0; // Converts from -1-1 to 0-1
		// if(speed < 0.05) speed = 0;
		double speed = 0;
		if(extraJoy.getRawButton(BUTTONS.BIG_JOY.LEFT_TINY_BUTTON))
			speed = -0.1;
		if(extraJoy.getRawButton(BUTTONS.BIG_JOY.RIGHT_TINY_BUTTON))
			speed = 0.1;
			
		spark1.set(speed);
		spark2.set(speed);
	}
}