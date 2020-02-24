package frc.robot;

class PORTS {
	public final static int FALCON_TALON = 1,
	INTAKE_TALON = 2,
	TALON_RF = 3,
	TALON_RB = 4,
	TALON_LF = 11,
	TALON_LB = 5,
	ROTATE_SPARK = 4,
	// DIO
	AXLE_ENCODER_A = 0,
	AXLE_ENCODER_B = 1,
	ROTATE_LIMIT = 7,
	// PWM
	HOOD_SERVO = 6;
}

class BUTTONS {
	// Logitech Game Controller
	// Buttons
	class GAMEPAD {
		public final static int A_BUTTON = 1,
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
		LEFT_JOY_X_AXIS = 0,
		LEFT_JOY_Y_AXIS = 1,
		LEFT_TRIGGER_AXIS = 2,
		RIGHT_TRIGGER_AXIS = 3,
		RIGHT_JOY_X_AXIS = 4,
		RIGHT_JOY_Y_AXIS = 5;
	}
	class BIG_JOY {
		public final static int TRIGGER = 1,
		UP_HANDLE_BUTTON = 3,
		DOWN_HANDLE_BUTTON = 2,
		LEFT_HANDLE_BUTTON = 4,
		RIGHT_HANDLE_BUTTON = 5;
	}
	class DRIVER_STATION {
		public final static int 
		// Axes
		LEFT_DIAL = 3,
		RIGHT_DIAL = 4,
		LEFT_SLIDER = 0,
		RIGHT_SLIDER = 1,
		// Buttons in the column (top to bottom)
		COL_BUTTON_1 = 0,
		COL_BUTTON_2 = 1,
		COL_BUTTON_3 = 2,
		COL_BUTTON_4 = 3,
		// Left and right on/off buttons
		LEFT_TOGGLE_BUTTON = 4,
		RIGHT_TOGGLE_BUTTON = 5,
		// On/off switches
		TOGGLE_SWITCH_1 = 6,
		TOGGLE_SWITCH_2 = 7,
		TOGGLE_SWITCH_3 = 8,
		// Momentary switches
		ROCKER_1_UP = 9,
		ROCKER_1_DOWN = 10,
		ROCKER_2_UP = 11,
		ROCKER_2_DOWN = 12,
		ROCKER_3_UP = 13,
		ROCKER_3_DOWN = 14;
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