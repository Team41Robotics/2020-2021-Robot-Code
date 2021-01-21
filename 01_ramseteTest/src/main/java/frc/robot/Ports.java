package frc.robot;

class PORTS {
	public final static int 
	// Talons
	FALCON_TALON = 15,
	INTAKE_TALON = 16,
	// Sparks
	SPARK_RF = 7,
	SPARK_RB = 8,
	SPARK_LF = 3,
	SPARK_LB = 2,
	ROTATE_SPARK = 12,
	ELEVATOR_SPARK_TOP = 6,
	ELEVATOR_SPARK_BOTTOM = 11,
	BALL_INTAKE_SPARK = 5,
	CLIMB_SPARK_1 = 4,
	CLIMB_SPARK_2 = 9,
	// DIO
	// Everything that has a value of 1 has not been found yet
	SHAFT_ENCODER_A = 2,
	SHAFT_ENCODER_B = 3,
	ROTATE_LIMIT_LEFT = 9,
	ROTATE_LIMIT_CENTER = 6,
	ROTATE_LIMIT_RIGHT = 8,
	ELEVATOR_LIMIT_TOP = 4,
	ELEVATOR_LIMIT_MIDDLE = 5,
	ELEVATOR_LIMIT_BOTTOM = 7,
	// PWM
	HOOD_SERVO = 2,
	// Pneumatics
	PCM = 1,
	INTAKE_SOL_FORWARD = 7,
	INTAKE_SOL_REVERSE = 0,
	CLIMB_SOL_FORWARD = 6,
	CLIMB_SOL_REVERSE = 1;
}

class BUTTONS {
	// Logitech Hand-Held Game Controller
	class GAMEPAD {
		public final static int 
		A_BUTTON = 1,
		B_BUTTON = 2,
		X_BUTTON = 3,
		Y_BUTTON = 4,
		L_BUMPER = 5,
		R_BUMPER = 6,
		BACK_BUTTON = 7,
		START_BUTTON = 8,
		L_JOY_CLICK = 9,
		R_JOY_CLICK = 10,
		// Axes
		L_JOY_X_AXIS = 0,
		L_JOY_Y_AXIS = 1,
		LEFT_TRIGGER_AXIS = 2,
		RIGHT_TRIGGER_AXIS = 3,
		R_JOY_X_AXIS = 4,
		R_JOY_Y_AXIS = 5;
	}
	// Logitech Attack 3 Joystick
	class BIG_JOY {
		public final static int 
		// Buttons
		TRIGGER = 1,
		UP_HANDLE_BUTTON = 3,
		DOWN_HANDLE_BUTTON = 2,
		LEFT_HANDLE_BUTTON = 4,
		RIGHT_HANDLE_BUTTON = 5,
		LEFT_FRONT_BUTTON = 6,
		LEFT_BACK_BUTTON = 7,
		RIGHT_FRONT_BUTTON = 11,
		RIGHT_BACK_BUTTON = 10,
		LEFT_TINY_BUTTON = 7,
		RIGHT_TINY_BUTTON = 8,
		// Axes
		X_AXIS = 0,
		Y_AXIS = 1,
		ROLLER = 2;
	}
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