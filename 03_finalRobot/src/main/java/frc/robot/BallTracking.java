package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.DriveConstants;


public class BallTracking(){

	private NetworkTable ball_tracker;
	public double l = DriveConstants.kTrackwidthMeters();

	public ballTracking(){
		ball_tracker = NetworkTableInstance.getDefault().getTable("ball_tracker");
	}

	public boolean getBallFound(){
		if(ball_tracker.getEntry("ball_found") == 1)
			return true;
		else	
			return false;
	}

	public double getLinearVelocity(){
		double k = 1.0;
		double r = ball_tracker.getEntry("radius");

		return k / r;
	}

	public double getAngularVelocity(){
		double b = 1.0;
		double d = ball_tracker.getEntry("displacement");

		return b * d;
	}

	public double getWheelVelocity(boolean direction){
		if(direction == true){ // Left Side
			return getLinearVelocity() - getAngularVelocity()*l/2.0;
		}
		else { // Right Side
			return getLinearVelocity() + getAngularVelocity()*l/2.0;
		}
	}
}
