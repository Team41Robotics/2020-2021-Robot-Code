package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

public final class Constants {

	public static final class Paths {
		public static final double StraightPath[][] ={{0.0, 0.0, 0, 0, },{0.1524, 0.0, 0.043481500739121874, 0.5520869496736904, },{0.3048, 0.0, 0.04348150073912183, 5, },{0.45720000000000005, 0.0, 0.04348150073912194, 5, },{0.6096, 0.0, 0.0434815007391219, 5, },{0.762, 0.0, 0.0434815007391219, 5, },{0.9144000000000001, 0.0, 0.043481500739121576, 5, },{1.0668, 0.0, 0.0434815007391219, 5, },{1.2192, 0.0, 0.043481500739121826, 5, },{1.3716000000000002, 0.0, 0.04348150073912175, 5, },{1.524, 0.0, 0.04348150073912232, 5, },{1.6764000000000001, 0.0, 0.04348150073912141, 5, },{1.8288000000000002, 0.0, 0.04351211643760231, 5, },{1.98098624, 0.0, 0.07762416920997846, 5, },{2.000213417984, 0.0, 0.6429022292722557, 1.5554464652144189, },{2.1524003414687747, 0.0, 0.04357338874341176, 1.650390121491758, },{2.3048000005463503, 0.0, 0.04348164751905537, 5, },{2.4572000000008742, 0.0, 0.043481500973967954, 5, },{2.6096000000000013, 0.0, 0.043481500739498545, 5, },{2.762, 0.0, 0.04349680320109542, 5, },{2.91429312, 0.0, 0.05575112366180091, 5, },{3.000106708992, 0.0, 0.0994457066619066, 5, },{3.152400170734387, 0.0, 0.04352740701638895, 5, },{3.304800000273175, 0.0, 0.04348157412899202, 5, },{3.4572000000004373, 0.0, 0.04348150085654639, 5, },{3.609600000000001, 0.0, 0.04348150073931168, 5, },{3.762, 0.0, 0.043481500739123304, 5, },{3.9144, 0.0, 0.04348150073911966, 5, },{4.0668, 0.0, 0.04348150073911663, 5, },{4.2192, 0.0, 0.04348150073912315, 5, },{4.3716, 0.0, 0.04348150073912997, 5, },{4.524, 0.0, 0.04348150073911648, 5, },{4.6764, 0.0, 0.043481500739123304, 5, },{4.8288, 0.0, 0.04348150073912315, 5, },{4.9812, 0.0, 0, 0, }};

	}
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

		public static final int kEncoderCPR = 12*60; //Unused
		public static final double kWheelDiameterMeters = 0.1524;
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