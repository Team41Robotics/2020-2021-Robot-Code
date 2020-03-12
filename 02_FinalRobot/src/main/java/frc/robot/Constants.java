package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

public final class Constants {
	public static final class DriveConstants {
		public static final int kLeftMotor1Port = 3;
		public static final int kLeftMotor2Port = 2;
		public static final int kRightMotor1Port = 7;
		public static final int kRightMotor2Port = 8;

		public static final boolean kLeftEncoderReversed = true;
		public static final boolean kRightEncoderReversed = false;

		public static final double kTrackwidthMeters = 0.7757; //.6341
		public static final DifferentialDriveKinematics kDriveKinematics =
				new DifferentialDriveKinematics(kTrackwidthMeters);

		public static final int kEncoderCPR = 2048; //Unused
		public static final double kWheelDiameterMeters = 0.2032;
		public static final double kEncoderDistancePerPulse =
				// Assumes the encoders are directly mounted on the wheel shafts
				(kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

		public static final boolean kGyroReversed = false;

		public static final double ksVolts = .151; //.94 .15
		public static final double kvVoltSecondsPerMeter = 2.36; //3.26 .0386
		public static final double kaVoltSecondsSquaredPerMeter = .471; //.461 .0101

		// Example value only - as above, this must be tuned for your drive!
		public static final double kPDriveVel = 0.5; //talon: 0.00299 WPILIB: 15.4
	}

	public static final class OIConstants {
		public static final int kDriverControllerPort = 1;
	}

	public static final class AutoConstants {
		public static final double kMaxSpeedMetersPerSecond = .5; //3
		public static final double kMaxAccelerationMetersPerSecondSquared = .5;

		// Reasonable baseline values for a RAMSETE follower in units of meters and seconds
		public static final double kRamseteB = 2; //2
		public static final double kRamseteZeta = .7; //.7
	}

	public static final class PDriveConstants {
		public static final int kLeftMotor1Port = 5;
		public static final int kLeftMotor2Port = 11;
		public static final int kRightMotor1Port = 4;
		public static final int kRightMotor2Port = 3;
	
		public static final boolean kLeftEncoderReversed = true;
		public static final boolean kRightEncoderReversed = false;
	
		public static final double kTrackwidthMeters = 0.6518; //.6341
		public static final DifferentialDriveKinematics kDriveKinematics =
			new DifferentialDriveKinematics(kTrackwidthMeters);
	
		public static final int kEncoderCPR = 4096; //1024???
		public static final double kWheelDiameterMeters = 0.1524;
		public static final double kEncoderDistancePerPulse =
			// Assumes the encoders are directly mounted on the wheel shafts
			(kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;
	
		public static final boolean kGyroReversed = false; //true for navX, false for realsense
	
		public static final double ksVolts = .764; //.94
		public static final double kvVoltSecondsPerMeter = 3.41; //3.26
		public static final double kaVoltSecondsSquaredPerMeter = .577; //.461
	
		public static final double kPDriveVel = .1; //talon: 0.00299 WPILIB: 15.4
	  }
	
	  public static final class PAutoConstants {
		public static final double kMaxSpeedMetersPerSecond = 1; //3
		public static final double kMaxAccelerationMetersPerSecondSquared = .5;
	
		// Reasonable baseline values for a RAMSETE follower in units of meters and seconds
		public static final double kRamseteB = 2; //2
		public static final double kRamseteZeta = 1.3; //.7
	  }
}