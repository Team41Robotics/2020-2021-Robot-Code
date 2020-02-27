package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

class Limelight {
	private Joystick controller;

	private NetworkTable limelight;
	private NetworkTableEntry tx, ty, ledMode, pipeline;

	private boolean isTracking;
	private boolean firstRun;

	private final boolean useHood;
	private boolean hoodDown = true;

	private final double a1 = Math.toRadians(9.5), // Angle of camera relative to the ground
	h1 = 0.705, // Height from geound to center of limelight (in meters)
	h2 = 2.30505; // Height to center of vision target (quarter of the height of the hexagon) (in meters)

	public Limelight() {
		controller = Robot.controller;
		limelight = NetworkTableInstance.getDefault().getTable("limelight");
		tx = limelight.getEntry("tx");
		ty = limelight.getEntry("ty");
		ledMode = limelight.getEntry("ledMode");
		pipeline = limelight.getEntry("pipeline");
		firstRun = true;

		this.useHood = Robot.useHood;
	}

	public void runLimelight() {
		if(firstRun)
			limelightInit();

		SmartDashboard.putNumber("Distance to Target", getDistance());
		SmartDashboard.putNumber("Limelight Target Shooter Speed", getShooterSpeed());
		updatePipeline();
		toggleTracking();
	}

	/**
	 * Initializes limelight LED configuration
	 */
	private void limelightInit() {
		ledMode.setNumber(1); // Set it to off
		pipeline.setNumber(0); // Sets network tables pipeline to Standard (1x Zoom)
		isTracking = false; // Since LED is off, we are not tracking
		firstRun = false;
	}

	/**
	 * Calculates the optimal shooter speed based on the distance calculated in <code>getDistance()</code>
	 * @return The output speed of the motor controller from 0 to 1
	 */
	public double getShooterSpeed() {
		// Quartic Relationship of distance to speed
		double a = 0.00741,
			b = -0.221,
			c = 2.66,
			d = -11.1,
			e = 79.7,
			x = getDistance();
		double speed = a*x*x*x*x + b*x*x*x + c*x*x + d*x + e;

		// If we're using the hood and it is down, set the speed to the data
		// that we obtained in the testing
		if(useHood && hoodDown) {
			a = 0.402;
			b = -3.6;
			c = 85.6;
			speed = a*x*x + b*x + c;
		}
		return speed/100.0; // Convert from percent to decimal value from 0 to 1
	}

	/**
	 * <a href="https://docs.limelightvision.io/en/latest/cs_estimating_distance.html">Source Link</a>
	 * <p><3</p><p><3</p><p><3</p><p><3</p><p><3</p><p><3</p><p><3</p><p><3</p><p><3</p><p><3</p><p><3</p>
	 * <p> Uses the limelight to estimate distance </p>
	 * @return Distance to target <<< IN METERS >>> 
	 */
	public double getDistance() {
		double a2 = Math.toRadians(ty.getDouble(0.0));
		double distance = (h2 - h1) / Math.tan(a1 + a2);
		return distance;
	}

	/**
	 * Changes the Limelight zoom when the D-Pad is pressed
	 * @param controller
	 */
	private void updatePipeline() {
		if(controller.getPOV() == 270) { // Left on the D-Pad sets zoom to 1x
			pipeline.setNumber(0);
			System.out.println("Zoom -> 1x");
		}
		if(controller.getPOV() == 90) { // Right on the D-Pad sets zoom to 2x
			pipeline.setNumber(1);
			System.out.println("Zoom -> 2x");
		}
		if(controller.getPOV() == 180) { // Down on the D-Pad sets zoom to 3x
			pipeline.setNumber(2);
			System.out.println("Zoom -> 3x");
		}
	}

	/**
	 * Turns the Limelight lights on or off
	 * @param controller
	 */
	private void toggleTracking() {
		if(controller.getRawButtonPressed(BUTTONS.GAMEPAD.START_BUTTON)) {
			ledMode.setNumber(isTracking ? 1 : 3);
			isTracking = !isTracking;
			System.out.println("Tracking -> " + (isTracking ? "On" : "Off"));
		}
	}

	public double getTX() {
		return tx.getDouble(0.0);
	}

	public boolean isTracking() {
		return isTracking;
	}

	public void setHoodDown(boolean hoodDown) {
		this.hoodDown = hoodDown;
	}
}