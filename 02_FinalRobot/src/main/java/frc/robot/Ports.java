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
	SHAFT_ENCODER_A = 0,
	SHAFT_ENCODER_B = 1,
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
	INTAKE_SOL_REVERSE = 0;
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
		L_JOY_Y_AXIS = 1,
		L_JOY_ROTATE = 2,
		R_JOY_X_AXIS = 0,
		R_JOY_Y_AXIS = 1,
		R_JOY_ROTATE = 2,
		// Buttons
		L_JOY_TRIGGER = 1,
		L_JOY_BUTTON_DOWN = 2,
		L_JOY_BUTTON_LEFT = 3,
		L_JOY_BUTTON_RIGHT = 4,
		R_JOY_TRIGGER = 1,
		R_JOY_BUTTON_DOWN = 2,
		R_JOY_BUTTON_LEFT = 3,
		R_JOY_BUTTON_RIGHT = 4,

		// Second Driverstation
		// Axes
		LEFT_DIAL = 3, // Manual Shoot Speed Control (0-MAX)
		RIGHT_DIAL = 4, // Lift Gear Shit Part 2
		LEFT_SLIDER = 0, // 
		RIGHT_SLIDER = 1, // 
		// Buttons in the column (top to bottom)
		COL_BUTTON_1 = 0, // Music Using Falcon
		COL_BUTTON_2 = 1, // 
		COL_BUTTON_3 = 2, // 
		COL_BUTTON_4 = 3, // Hood Up/Down Toggle
		// Left and right on/off buttons
		LEFT_TOGGLE_BUTTON = 4, // Shooter Speed Auto/Manual Toggle
		RIGHT_TOGGLE_BUTTON = 5, // Lift Gear Shift Part 1
		// Momentary switches
		ROCKER_L_UP = 6, // Turret Clockwise
		ROCKER_L_DOWN = 7, // Turret Counterclockwise
		ROCKER_M_UP = 8, // Elevator Top Up
		ROCKER_M_DOWN = 9, // Elevator Top Down
		ROCKER_R_UP = 10, // Elevator Bottom Up
		ROCKER_R_DOWN = 11, // Elevator Bottom Down
		// On/off switches
		TOGGLE_SWITCH_L = 12, // Limelight On/Off
		TOGGLE_SWITCH_M = 13, // Hood Auto/Manual
		TOGGLE_SWITCH_R = 14, // 
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