package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;


class Intake {
	private DoubleSolenoid solenoidLeft, solenoidRight, solenoidHood;
	private boolean leftTrigger, rightTrigger;	

	public static Joystick leftJoy = new Joystick(3);
	public static Joystick rightJoy = new Joystick(4);

	private TalonSRX ballIntake;

	public Intake() {
	

		solenoidLeft = new DoubleSolenoid(PORTS.PCM, PORTS.LEFT_SOLENOID_FORWARD, PORTS.LEFT_SOLENOID_BACKWARD);
		solenoidRight = new DoubleSolenoid(PORTS.PCM, PORTS.RIGHT_SOLENOID_FORWARD, PORTS.RIGHT_SOLENOID_BACKWARD);
		solenoidHood = new DoubleSolenoid(PORTS.PCM, PORTS.HOOD_SOLENOID_FORWARD, PORTS.HOOD_SOLENOID_BACKWARD);
		ballIntake = new TalonSRX(PORTS.INTAKE_TALON);

		intakeInit();
	}

	private void intakeInit() {
		System.out.println("Solenoid Reverse");
		solenoidLeft.set(Value.kReverse);
		solenoidRight.set(Value.kReverse);
		solenoidHood.set(Value.kReverse);
		//solenoidLeft.set(Value.kForward);
		//solenoidRight.set(Value.kForward);
		//solenoidHood.set(Value.kForward);
		leftTrigger = false;
		rightTrigger = false;
	}

	public void periodic() {
		togglePistons();
		//toggleIntake();
	}

	private void togglePistons() {
		if(leftJoy.getRawButtonPressed(1)){
			if(!leftTrigger){ 
				System.out.println("forward");
				leftTrigger = true;
				solenoidLeft.set(Value.kForward);
				solenoidRight.set(Value.kForward);
				solenoidHood.set(Value.kForward);
			}
			else {
				leftTrigger = false;
				System.out.println("reverse");
				solenoidLeft.set(Value.kReverse);
				solenoidRight.set(Value.kReverse);
				solenoidHood.set(Value.kReverse);
			}
		}
	}

	private void toggleIntake() {
		if(rightJoy.getRawButtonPressed(1)){
			if(!rightTrigger){
				rightTrigger = true;
				ballIntake.set(ControlMode.PercentOutput, 0.60); 
			}
			else{
				rightTrigger = false;
				ballIntake.set(ControlMode.PercentOutput, 0); 
			}
		}
	}

	private void auton() {
		
	}

}