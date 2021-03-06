package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

class Limelight {
	private Joystick driverstation;

	private NetworkTable limelight;
	private NetworkTableEntry tx, ty, ledMode, pipeline;

	private boolean isTracking;

	private final boolean useHood;
	private boolean hoodDown = true;

	private final double a1 = Math.toRadians(13.5), // Angle of camera relative to the ground
	h1 = 0.7, // Height from geound to center of limelight (in meters)
	h2 = 2.30505; // Height to center of vision target (quarter of the height of the hexagon) (in meters)

	private final boolean flywheel = false;

	public Limelight() {
		driverstation = Robot.driverstation;
		limelight = NetworkTableInstance.getDefault().getTable("limelight");
		tx = limelight.getEntry("tx");
		ty = limelight.getEntry("ty");
		ledMode = limelight.getEntry("ledMode");
		pipeline = limelight.getEntry("pipeline");

		this.useHood = Robot.useHood;

		// Initialize LED configuration
		ledMode.setNumber(1); // Set it to off
		pipeline.setNumber(0); // Sets network tables pipeline to Standard (1x Zoom)
		isTracking = false; // Since LED is off, we are not tracking
	}

	public void auton(){
		ledMode.setNumber(3);
		isTracking = true;

		// SmartDashboard.putNumber("Distance to Target", getDistance());
		// SmartDashboard.putNumber("Limelight Target Shooter Speed", getTargetShooterSpeed());
	}

	public void periodic() {
		updatePipeline();
		updateTracking();

		// SmartDashboard.putNumber("Distance to Target", getDistance());
		// SmartDashboard.putNumber("Limelight Target Shooter Speed", getTargetShooterSpeed());
	}

	/**
	 * Calculates the optimal shooter speed based on the distance calculated in <code>getDistance()</code>
	 * @return The output speed of the motor controller from 0 to 1
	 */
	public double getTargetShooterSpeed() {
		double speed = 0;
		if(flywheel) {
			// Quartic Relationship of distance to speed
			double a = 0.00741,
				b = -0.221,
				c = 2.66,
				d = -11.1,
				e = 79.7,
				x = getDistance();
			speed = a*x*x*x*x + b*x*x*x + c*x*x + d*x + e;

			// If we're using the hood and it is down, set the speed to the data
			// that we obtained in the testing
			if(useHood && hoodDown) {
				a = 0.402;
				b = -3.6;
				c = 85.6;
				speed = a*x*x + b*x + c;
			}
		}
		else {
			double a = 6.82,
				b = -52.7,
				c = 175,
				x = getDistance();
			speed = a*x*x + b*x + c;

			// If we're using the hood and it is down, set the speed to the data
			// that we obtained in the testing
			if(useHood && hoodDown) {
				a = -0.6;
				b = 10.2;
				c = 43.3;
				speed = a*x*x + b*x + c;
			}
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
	 */
	private void updatePipeline() {
		int pov = driverstation.getPOV(0)/45;
		int lime = pov & 0b001;
		pipeline.setNumber(lime);
	}

	/**
	 * Turns the Limelight lights on or off
	 */
	private void updateTracking() {
		if(driverstation.getRawButton(BUTTONS.DRIVER_STATION.TOGGLE_SWITCH_M)) {
			if(!isTracking) { // So we don't repeatedly set the mode to on
				ledMode.setNumber(3);
				isTracking = true;
			}
		}
		else {
			if(isTracking) { // So we don't repeatedly set the mode to off
				ledMode.setNumber(1);
				isTracking = false;
			}
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