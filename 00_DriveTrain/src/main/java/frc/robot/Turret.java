package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;

class Turret {
    private TalonSRX turret;

	private Encoder turretEnc;

	private DigitalInput turretLimit;

	private final int TURRET_ENC_MAX = 775_000; // Found experimentally
	private final int TURRET_ENC_MIN = 50_000; // So the limit switch doesn't get slammed
	private boolean turretZeroed = false;
	
	private Vision vis;

    public Turret(Vision vis) {
        turret = new TalonSRX(PORTS.TURRET_TALON);
		// turretEnc = new Encoder(PORTS.TURRET_TALON);
		turretEnc = new Encoder(PORTS.TURRET_ENCODER_A, PORTS.TURRET_ENCODER_B, false, Encoder.EncodingType.k4X);
		turretEnc.reset();
		turretLimit = new DigitalInput(PORTS.TURRET_LIMIT);

		this.vis = vis;
    }

    public void zeroTurret() {
        if(turretLimit.get()) {
			turretEnc.reset();
            turretZeroed = true;
			turret.set(ControlMode.PercentOutput, 0);
			System.out.println("Turret zeroed");
        }
        else {
			turret.set(ControlMode.PercentOutput, -0.075);
			System.out.println("Zeroing");
		}
    }

    public void controllerMove(Joystick controller) {
        if(!turretZeroed) { // Don't move if not zeroed yet
			zeroTurret();
			return;
		}
		
        double speedMultiplier = 0.25;
		if(controller.getRawButton(BUTTONS.TURRET_SLOW) == true) {
			speedMultiplier *= 0.5;
		}

		if(turretLimit.get()) {
			turretEnc.reset();
			turretZeroed = true;
		}
		double leftTrigger = controller.getRawAxis(2); // Left trigger axis - Counterclockwise
		double rightTrigger = controller.getRawAxis(3); // Right trigger axis - Clockwise
		boolean useVision = true;
		// Counterclockwise is positive speed, Clockwise is negative speed
		double speed = 0;
		if(leftTrigger > 0) {
			if(turretEnc.getRaw() < TURRET_ENC_MAX)
				speed = leftTrigger*leftTrigger*leftTrigger*speedMultiplier;
		}
		else if(rightTrigger > 0) {
			if(!turretZeroed || turretEnc.getRaw() > TURRET_ENC_MIN)
				speed = -rightTrigger*rightTrigger*rightTrigger*speedMultiplier;
		}
		else {
			if(vis.isTracking() && useVision) {
				double tolerance = 0.5; // In degrees
				double tx = vis.getTX();
				if(tx > tolerance) {
					System.out.println("Rotate turret clockwise");
					if(!turretZeroed || turretEnc.getRaw() > TURRET_ENC_MIN)
						speed = -0.1;
				}
				else if(tx < -tolerance) {
					System.out.println("Rotate turret counterclockwise");
					if(turretEnc.getRaw() < TURRET_ENC_MAX)
						speed = 0.1;
				}
			}
		}
		// System.out.println(turretEnc.getRaw() + " | " + turretLimit.get());
		
		// Positive values are left of target, negative are right
		turret.set(ControlMode.PercentOutput, speed);
	}
}