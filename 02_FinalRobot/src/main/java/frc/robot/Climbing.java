package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;

class Climbing {
	private final MotorType kBrushless = MotorType.kBrushless;

	private Joystick controller;
	private CANSparkMax spark1, spark2;

	public Climbing() {
		controller = Robot.controller;
		spark1 = new CANSparkMax(PORTS.CLIMB_SPARK_1, kBrushless);
		spark2 = new CANSparkMax(PORTS.CLIMB_SPARK_2, kBrushless);
	}

	public void controllerMove() {
		double speed = controller.getRawAxis(BUTTONS.BIG_JOY.ROLLER);
		speed = (speed + 1.0) / 2.0; // Converts from -1-1 to 0-1
		if(speed < 0.05) speed = 0;
		spark1.set(speed);
		spark2.set(speed);
	}
}