package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

class Vision {
	private NetworkTable table;
	private NetworkTableEntry tx, ty, ledMode, pipeline;

	private boolean isTracking;
	private boolean firstRun;

	private final double a1 = Math.toRadians(11.0), // Angle of camera relative to the ground
	h1 = 0.705, // Height from geound to center of limelight (in meters)
	h2 = 2.30505; // Height to center of vision target (quarter of the height of the hexagon) (in meters)

	public Vision() {
		table = NetworkTableInstance.getDefault().getTable("limelight");
		tx = table.getEntry("tx");
		ty = table.getEntry("ty");
		ledMode = table.getEntry("ledMode");
		pipeline = table.getEntry("pipeline");
		firstRun = true;
	}

	public void runVision(Joystick controller) {
		if(firstRun) {
			ledMode.setNumber(1); // Set it to off
			pipeline.setNumber(0); // Set it to Standard
			isTracking = false; // Since LED is off, we are not tracking
			firstRun = false;
		}

		double x = tx.getDouble(0.0);
		double y = ty.getDouble(0.0);

		SmartDashboard.putNumber("LimelightX", x);
		SmartDashboard.putNumber("LimelightY", y);

		if(controller.getRawButtonPressed(BUTTONS.GAMEPAD.START_BUTTON)) {
			ledMode.setNumber(isTracking ? 1 : 3);
			isTracking = !isTracking;
		}
		SmartDashboard.putNumber("Distance to Target", getDistance());
		updatePipeline(controller);
	}

	/**
	 * Calculates the optimal shooter speed based on the distance calculated in <code>getDistance</code>
	 * @return The output speed of the motor controller from -1 to 1
	 */
	public double getShooterSpeed() {
		// Quadratic Relationship of distance to speed
		// speed = ax^2 + bx + c
		double a = 0.58,
			b = -3.27,
			c = 69.7,
			x = getDistance();
		double speed = a*x*x + b*x + c;
		return speed/100.0;
	}

	public double getHoodAngle() {
		double x = getDistance();
		double k = 1.0; //Some experimentally determined variable
		return k*x; 
	}

	/**
	 * <a href="https://docs.limelightvision.io/en/latest/cs_estimating_distance.html">Source Link</a>
	 * <p><3</p><p><3</p><p><3</p><p><3</p><p><3</p><p><3</p><p><3</p><p><3</p><p><3</p><p><3</p><p><3</p>
	 * <p> Uses the limelight to estimate distance </p>
	 * @return Distance to target <<< IN METERS >>> 
	 */
	private double getDistance() {
		double a2 = Math.toRadians(ty.getDouble(0.0));
		double distance = (h2 - h1) / Math.tan(a1 + a2);
		return distance;
	}

	/**
	 * Toggles the pipeline between standard and 2x_zoom when D-Pad is pressed
	 * @param controller
	 */
	private void updatePipeline(Joystick controller) {
		if(controller.getPOV() == 270) { // Left on the D-Pad sets zoom to 1x
			pipeline.setNumber(0);
		}
		if(controller.getPOV() == 90) { // Right on the D-Pad sets zoom to 2x
			pipeline.setNumber(1);
		}
		if(controller.getPOV() == 180) { // Down on the D-Pad sets zoom to 3x
			pipeline.setNumber(2);
		}
	}

	public double getTX() {
		return tx.getDouble(0.0);
	}

	public boolean isTracking() {
		return isTracking;
	}
}