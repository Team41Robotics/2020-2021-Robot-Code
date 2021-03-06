package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

class Hood {
	private Joystick driverstation;
	private Servo hoodServo;

	private final double maxAngle = 120;
	private final double minAngle = 0;
	private double hoodAngle;
	private boolean hoodDown;

	private Limelight lime;

	public Hood() {
		driverstation = Robot.driverstation;
		hoodServo = new Servo(PORTS.HOOD_SERVO);
		hoodDown = false;
		hoodAngle = hoodDown ? minAngle : maxAngle;
		hoodServo.setAngle(hoodAngle);

		// this.lime = Robot.lime;
		lime.setHoodDown(hoodDown);
	}

	public void periodic() {
		setHoodPos();
		SmartDashboard.putBoolean("Hood Down", hoodDown);
	}

	private void setHoodPos() {
		double distance = lime.getDistance();

		boolean toggle = false;
		boolean autoAdjust = driverstation.getRawButton(BUTTONS.DRIVER_STATION.TOGGLE_SWITCH_R);

		if(autoAdjust && lime.isTracking()) {
			// Only auto-adjust if we are not already adjusted
			// and are at the right distance
			if(hoodDown)
				toggle = distance < 4;
			else
				toggle = distance > 4.25;
		}
		else // If we're not auto adjusting, use the controller
			toggle = driverstation.getRawButtonPressed(BUTTONS.DRIVER_STATION.COL_BUTTON_4);

		if(toggle)
			toggleHoodDown();
	}

	private void toggleHoodDown() {
		hoodDown = !hoodDown;
		lime.setHoodDown(hoodDown);

		System.out.println("Hood -> " + (hoodDown ? "Down" : "Up"));

		hoodAngle = hoodDown ? minAngle : maxAngle;
		hoodServo.setAngle(hoodAngle);
	}
}