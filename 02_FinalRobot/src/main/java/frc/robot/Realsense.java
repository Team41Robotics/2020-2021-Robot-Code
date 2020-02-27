package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;

class Realsense {
	private NetworkTable realsense;
	private NetworkTableEntry angleReal, angleRobo, xRobo, yRobo;

	private boolean firstRun;

	private double xStart, yStart, xFinal, yFinal, angleStart, angleFinal;

	public Realsense() {
		realsense = NetworkTableInstance.getDefault().getTable("realsense");

		xRobo = realsense.getEntry("xR"); //X coordinate of robot in relation to field
		yRobo = realsense.getEntry("yR"); //Y coordinate of robot in relation to field
		angleRobo = realsense.getEntry("angleR"); //Angle of robot in relation to field
		angleReal = realsense.getEntry("angle"); //Angle of Realsense in relation to start
	
		firstRun = true;
	}

	public void runRealsense(Joystick controller) {
		if(firstRun)
			realsenseInit();
	}

	public void realsenseInit(){
		xStart = xRobo.getDouble(0.0);
		yStart = yRobo.getDouble(0.0);
		angleStart = angleRobo.getDouble(0.0);

		if(xFinal == 0.0)
			xFinal = xStart;
		if(yFinal == 0.0)
			yFinal = yStart;

		firstRun = false;
	}

	public double getAngleToDestination() {
		// double x1 = xRobo.getDouble(0.0);
		// double y1 = yRobo.getDouble(0.0);
		// double currentAngle = angleRobo.getDouble(0.0);

		double Dx = xFinal - xStart;
		double Dy = yFinal - yStart;

		angleFinal = Math.atan( Dy/Dx );
		
		return angleFinal - angleStart; //in radians
	}

	public double getDistanceToDestination() {
		double Dx = xFinal - xStart;
		double Dy = yFinal - yStart;

		// Just plug into the distance formula
		double distance = Math.sqrt(Math.pow(Dx,2) + Math.pow(Dy,2)); 

		return distance;
	}

	public void setDestination(double x, double y){
		xFinal = x;
		yFinal = y;
	}

}