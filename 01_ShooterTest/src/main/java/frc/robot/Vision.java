package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

class Vision {
    private NetworkTable table;
    private NetworkTableEntry tx;

    private NetworkTableEntry ledMode;
    private boolean isTracking;

    private boolean firstRun;

    public Vision() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx");
        ledMode = table.getEntry("ledMode");
        firstRun = true;
    }

    public void runVision(Joystick controller) {
        if(firstRun) {
            ledMode.setNumber(1); // Set it to off
            isTracking = false; // Since LED is off, we are not tracking
            firstRun = false;
        }

        double x = tx.getDouble(0.0);

        SmartDashboard.putNumber("LimelightX", x);

        if(controller.getRawButtonPressed(BUTTONS.START_BUTTON)) {
            ledMode.setNumber(isTracking ? 1 : 3);
            isTracking = !isTracking;
        }
    }

    public double getTX() {
        return tx.getDouble(0.0);
    }

    public boolean isTracking() {
        return isTracking;
    }
}