package frc.robot;

import java.text.DecimalFormat;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;

class Hood {
	private DecimalFormat df;

	private Servo hoodServo;

	private boolean autoAdjust = true;
	private double hoodAngle = 0;
	private final double increment = 10;
	private final double maxAngle = 110;
	private final double minAngle = 0;

	private Limelight lime;

	public Hood(Limelight lime) {
		hoodServo = new Servo(PORTS.HOOD_SERVO);

		df = new DecimalFormat("0.##");

		this.lime = lime;
	}

	public void controllerMove(Joystick controller) {
		// Use trigger to toggle auto adjust
		if(controller.getRawButtonPressed(BUTTONS.BIG_JOY.TRIGGER)) {
			autoAdjust = !autoAdjust;
			System.out.println("Hood Auto Adjust -> " + (autoAdjust ? "On" : "Off"));
		}

		maxHoodPos(controller);
		if(!autoAdjust || !lime.isTracking())
			incrementHoodPos(controller);	
	}

	private void maxHoodPos(Joystick controller) {
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
			up = controller.getRawButtonPressed(BUTTONS.BIG_JOY.RIGHT_HANDLE_BUTTON);
			down = controller.getRawButtonPressed(BUTTONS.BIG_JOY.LEFT_HANDLE_BUTTON);
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
	private void incrementHoodPos(Joystick controller) {
		boolean upIncrement = controller.getRawButtonPressed(BUTTONS.BIG_JOY.UP_HANDLE_BUTTON);
		boolean downIncrement = controller.getRawButtonPressed(BUTTONS.BIG_JOY.DOWN_HANDLE_BUTTON);
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