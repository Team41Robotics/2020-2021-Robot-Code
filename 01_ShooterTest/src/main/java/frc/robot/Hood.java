package frc.robot;

import java.text.DecimalFormat;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;

class Hood {
	private DecimalFormat df;

	private Servo hoodServo;

	private double hoodAngle = 0;
	private final double increment = 10;
	private final double maxAngle = 180;
	private final double minAngle = 0;

	private Vision vis;

	public Hood(Vision vis) {
		hoodServo = new Servo(0);

		df = new DecimalFormat("0.##");

		this.vis = vis;
	}

	public void controllerMove(Joystick controller) {
		boolean up = controller.getRawButtonPressed(BUTTONS.BIG_JOY.RIGHT_HANDLE_BUTTON);
		boolean down = controller.getRawButtonPressed(BUTTONS.BIG_JOY.LEFT_HANDLE_BUTTON);
		boolean upIncrement = controller.getRawButtonPressed(BUTTONS.BIG_JOY.UP_HANDLE_BUTTON);
		boolean downIncrement = controller.getRawButtonPressed(BUTTONS.BIG_JOY.DOWN_HANDLE_BUTTON);
		final double initAngle = hoodAngle;

		if(up) {
			hoodAngle = maxAngle;
		}
		else if (down) {
			hoodAngle = minAngle;
		}
		else if(upIncrement && hoodAngle <= maxAngle - increment) {
			hoodAngle += increment;
		}
		else if (downIncrement && hoodAngle >= minAngle + increment) {
			hoodAngle -= increment;
		}

		// hoodAngle = vis.getHoodAngle(); //currently sets hood angle to distance <3

		if(hoodAngle <= minAngle)
			hoodAngle = minAngle;
		else if(hoodAngle >= maxAngle)
			hoodAngle = maxAngle;

		if(hoodAngle != initAngle) {
			System.out.println("Setting hood position to " + df.format(hoodAngle) + "deg");
			hoodServo.setAngle(hoodAngle);
		}
	}
}