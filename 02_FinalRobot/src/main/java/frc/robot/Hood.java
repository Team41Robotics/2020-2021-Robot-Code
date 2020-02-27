package frc.robot;

import java.text.DecimalFormat;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;

class Hood {
	private DecimalFormat df;

	private Joystick extraJoy;
	private Servo hoodServo;

	private boolean autoAdjust = true;
	private double hoodAngle = 0;
	private final double increment = 10;
	private final double maxAngle = 110;
	private final double minAngle = 0;

	private Limelight lime;

	public Hood() {
		df = new DecimalFormat("0.##");

		extraJoy = Robot.extraJoy;
		hoodServo = new Servo(PORTS.HOOD_SERVO);
		hoodServo.setAngle(minAngle);


		this.lime = Robot.lime;
	}

	public void controllerMove() {
		// Use trigger to toggle auto adjust
		/* if(extraJoy.getRawButtonPressed(BUTTONS.BIG_JOY.TRIGGER)) {
			autoAdjust = !autoAdjust;
			System.out.println("Hood Auto Adjust -> " + (autoAdjust ? "On" : "Off"));
		} */

		maxHoodPos();
		if(!autoAdjust || !lime.isTracking())
			incrementHoodPos();	
	}

	private void maxHoodPos() {
		boolean up = false, down = false;
		double distance = lime.getDistance();

		boolean change = false;

		if(autoAdjust && lime.isTracking()) {
			// Only auto-adjust if we are not already adjusted
			// and are at the right distance
			up = distance < 4 && hoodAngle != maxAngle;
			down = distance > 4.25 && hoodAngle != 0;
		}
		else { // If we're not auto adjusting, use the controller
			up = extraJoy.getRawButtonPressed(BUTTONS.BIG_JOY.RIGHT_HANDLE_BUTTON);
			down = extraJoy.getRawButtonPressed(BUTTONS.BIG_JOY.LEFT_HANDLE_BUTTON);
		}

		// Sets the hood to the up position so the ball does not hit it
		if(up) {
			hoodAngle = maxAngle;
			lime.setHoodDown(false); // Adjusts limelight shot speed calculation
			change = true;
		}
		// Sets the hood to the down position so that it changes the ball angle
		else if (down) {
			hoodAngle = minAngle;
			lime.setHoodDown(true); // Adjusts limelight shot speed calculation
			change = true;
		}

		if(change) {
			System.out.println("Hood Pos -> " + (up ? "Up" : "Down"));
			hoodServo.setAngle(hoodAngle);
		}
	}
	private void incrementHoodPos() {
		boolean upIncrement = extraJoy.getRawButtonPressed(BUTTONS.BIG_JOY.UP_HANDLE_BUTTON);
		boolean downIncrement = extraJoy.getRawButtonPressed(BUTTONS.BIG_JOY.DOWN_HANDLE_BUTTON);
		boolean change = false;

		if(upIncrement && hoodAngle <= maxAngle - increment) {
			hoodAngle += increment;
			change = true;
		}
		else if (downIncrement && hoodAngle >= minAngle + increment) {
			hoodAngle -= increment;
			change = true;
		}

		if(change) {
			System.out.println("Hood Pos -> " + df.format(hoodAngle) + "°");
			hoodServo.setAngle(hoodAngle);
		}
	}
}