package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

class Turret {
    private TalonSRX turret;
	private Encoder turretEnc;
	private DigitalInput turretLimit;

	private final double TURRET_ENC_MAX = 775_000; // Found experimentally
	private final double TURRET_ENC_MIN = 50_000; // So the limit switch doesn't get slammed
	private boolean turretZeroed = false;

	// PID
	private PIDController pid;
	private long startTime = 0;

	private Vision vis;

    public Turret(Vision vis) {
        turret = new TalonSRX(PORTS.TURRET_TALON);
		turretEnc = new Encoder(PORTS.TURRET_ENCODER_A, PORTS.TURRET_ENCODER_B);
		turretEnc.reset();
		turretLimit = new DigitalInput(PORTS.TURRET_LIMIT);

		pid = new PIDController(0.4, 0, 0);

		this.vis = vis;
    }

    public void zeroTurret() {
        if(turretLimit.get()) {
			turretEnc.reset();
            turretZeroed = true;
			turret.set(ControlMode.PercentOutput, 0);
			System.out.println("Turret zeroed");
        }
        else{
			turret.set(ControlMode.PercentOutput, -0.075);
			System.out.println("Zeroing");
		}
    }

    public void controllerMove(Joystick controller) {
        // if(!turretZeroed) { // Don't move if not zeroed yet
		// 	zeroTurret();
		// 	return;
		// }
		
        double speedMultiplier = 0.25;
		if(controller.getRawButton(BUTTONS.A_BUTTON) == true) { // Half speed when pressing 'a'
			speedMultiplier *= 0.5;
		}

		if(turretLimit.get()) { // Zero encoder if limit is hit
			turretEnc.reset();
			turretZeroed = true;
		}
		double leftTrigger = controller.getRawAxis(BUTTONS.LEFT_TRIGGER_AXIS); // Counterclockwise
		double rightTrigger = controller.getRawAxis(BUTTONS.RIGHT_TRIGGER_AXIS); // Clockwise
		boolean useVision = true;

		// Counterclockwise is positive speed, Clockwise is negative speed
		double speed = 0;
		double motorDrive = 0;
		double encoderValAdjusted = turretEnc.getRaw()/TURRET_ENC_MAX;
		if(startTime == 0) {
			if(leftTrigger > 0) { // Counterclockwise
				if(turretEnc.getRaw() < TURRET_ENC_MAX) {
					speed = leftTrigger*leftTrigger*leftTrigger*speedMultiplier;
					motorDrive = pid.calculate(encoderValAdjusted, leftTrigger);
				}
			}
			else if(rightTrigger > 0) { //Clockwise
				if(!turretZeroed || turretEnc.getRaw() > TURRET_ENC_MIN) {
					speed = -rightTrigger*rightTrigger*rightTrigger*speedMultiplier;
					motorDrive = pid.calculate(encoderValAdjusted, -rightTrigger);
				}
			}
			else if(vis.isTracking() && useVision) {
				double tolerance = 0.5; // In degrees
				double tx = vis.getTX();
				if(tx > tolerance) {
					if(!turretZeroed || turretEnc.getRaw() > TURRET_ENC_MIN)
						speed = -0.1;
				}
				else if(tx < -tolerance) {
					if(turretEnc.getRaw() < TURRET_ENC_MAX)
						speed = 0.1;
				}
			}

			// Positive values are left of target, negative are right
			turret.set(ControlMode.PercentOutput, speed);
		}
		

		double posError = pid.getPositionError();
		if(rightTrigger != 0) {
			System.out.println(rightTrigger);
			System.out.println(encoderValAdjusted);
			System.out.println(posError);
			System.out.println(motorDrive);
			System.out.println("--------------------");

			SmartDashboard.putNumber("Position Error", posError);
			SmartDashboard.putNumber("Motor Value", motorDrive);
		}
		
		if(controller.getRawButtonPressed(BUTTONS.R_BUMPER)) {
			startTime = System.nanoTime();
			turret.set(ControlMode.PercentOutput, -0.4);
		}
		if(controller.getRawButtonPressed(BUTTONS.L_BUMPER)) {
			startTime = System.nanoTime();
			turret.set(ControlMode.PercentOutput, 0.4);
		}
		if(System.nanoTime() - startTime > 500_000_000 && startTime != 0) {
			turret.set(ControlMode.PercentOutput, 0);
			startTime = 0;
		}
	}
}