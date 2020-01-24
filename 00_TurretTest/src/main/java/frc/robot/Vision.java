package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

class Vision {
    private NetworkTable table;
    private NetworkTableEntry tx;
    private NetworkTableEntry ty;
    private NetworkTableEntry ta;

    private NetworkTableEntry ledMode;
    private boolean isTracking;

    private boolean firstRun;

    public Vision() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        ledMode = table.getEntry("ledMode");
        firstRun = true;
    }

    public void runVision(Joystick controller) {
        if(firstRun) {
            ledMode.setNumber(1); // Set it to off
            isTracking = false; // Since LED is off, we are not tracking
            firstRun = false;
        }

        //read values periodically
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);

        //post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);

        if(controller.getRawButtonPressed(BUTTONS.B_BUTTON)) {
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