package frc.robot;

class PORTS {
	public final static int 
	// Talons for the new robot
	INTAKE_TALON = 14,
	SHOOTER_FALCON = 11,
	SHOOTER_TALON = 13,
	SPINDEXER_FALCON = 12,

	FALCON_TALON = 0,
	// Sparks for the new robot
	SPARK_RF = 5,
	SPARK_RB = 4,
	SPARK_LF = 15,
	SPARK_LB = 1,
	
	// for the new robot
	LITTLE_OMNI_SPARK = 3,
	BIG_OMNI_SPARK = 10,
	ELEVATOR_SPARK_LEFT = 13,
	ELEVATOR_SPARK_RIGHT = 14,
	ROTATE_SPARK = 2,
	// DIO
	ROTATE_LIMIT_LEFT = 2,
	ROTATE_LIMIT_CENTER = 3,
	ROTATE_LIMIT_RIGHT = 1,
	// PWM
	HOOD_SERVO = 2,
	// PCM
	LEFT_SOLENOID_FORWARD = 3,
	LEFT_SOLENOID_BACKWARD = 2,

	RIGHT_SOLENOID_FORWARD = 5,
	RIGHT_SOLENOID_BACKWARD = 4,

	HOOD_SOLENOID_FORWARD = 1,
	HOOD_SOLENOID_BACKWARD = 0,
	PCM = 15;
}

class BUTTONS {
	// Custom Driver Station
	class DRIVER_STATION {
		public final static int 
		// First Driverstation
		// Axes
		L_JOY_X_AXIS = 0,
		L_JOY_Y_AXIS = 1, // Tank Drive Left
		L_JOY_ROTATE = 2,
		R_JOY_X_AXIS = 0,
		R_JOY_Y_AXIS = 1, // Tank Drive Right
		R_JOY_ROTATE = 2,
		// Buttons
		L_JOY_TRIGGER = 1, // Intake Arm Forward/Reverse
		L_JOY_BUTTON_DOWN = 2,
		L_JOY_BUTTON_LEFT = 3, // Emergency stop drivetrain
		L_JOY_BUTTON_RIGHT = 4, // Speed multiplier down
		R_JOY_TRIGGER = 1, // Intake Arm On/Off
		R_JOY_BUTTON_DOWN = 2,
		R_JOY_BUTTON_LEFT = 3, // Speed multiplier up
		R_JOY_BUTTON_RIGHT = 4,

		// Second Driverstation
		// Axes
		LEFT_DIAL = 4, // Manual/Auto Intake
		RIGHT_DIAL = 3, // Lift Gear Shit Part 2
		LEFT_SLIDER = 0, // Manual Shoot Speed Control (0-MAX)
		RIGHT_SLIDER = 1, // 
		// Buttons in the column (top to bottom)
		COL_BUTTON_1 = 1, // Stop shooter
		COL_BUTTON_2 = 2, // Set speed to limelight target speed
		COL_BUTTON_3 = 3, // Engage the lift piston
		COL_BUTTON_4 = 4, // Hood Up/Down Toggle
		// Left and right on/off buttons
		LEFT_TOGGLE_BUTTON = 5, // Shooter Speed Auto/Manual Toggle
		RIGHT_TOGGLE_BUTTON = 6, // Lift Gear Shift Part 1
		// Momentary switches
		ROCKER_L_UP = 7, // Turret Clockwise
		ROCKER_L_DOWN = 8, // Turret Counterclockwise
		ROCKER_M_UP = 9, // Elevator Top Up
		ROCKER_M_DOWN = 10, // Elevator Top Down
		ROCKER_R_UP = 11, // Elevator Bottom Up
		ROCKER_R_DOWN = 12, // Elevator Bottom Down
		// On/off switches
		TOGGLE_SWITCH_L = 13, // Shooter Intake On/Off
		TOGGLE_SWITCH_M = 14, // Limelight On/Off
		TOGGLE_SWITCH_R = 15, // Hood Auto/Manual
		// Touch Displays
		TOP_DISPLAY = 0,
		BOTTOM_DISPLAY = 1;
	}
}

class COORDINATES {
	public final static Point
		START_TOP = new Point(0,0),
		START_MIDDLE = new Point(0,0),
		START_BOTTOM = new Point(0,0),
		GOAL = new Point(0,0),
		TRENCH = new Point(0,0),
		DISPENSER = new Point(0,0);
}

class Point {
	public final double x, y;

	public Point(double x, double y) {
		this.x = x;
		this.y = y;
	}
}