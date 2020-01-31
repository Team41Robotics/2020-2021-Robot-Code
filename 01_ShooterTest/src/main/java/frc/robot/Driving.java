package frc.robot;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Joystick;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Driving {
	private WPI_TalonSRX talonRF, talonRB, talonLF, talonLB;
	private SpeedControllerGroup leftSpeedCG, rightSpeedCG;
	private DifferentialDrive difDrive;

	public Driving() {
		talonRF = new WPI_TalonSRX(PORTS.TALON_RF);
		talonRB = new WPI_TalonSRX(PORTS.TALON_RB);
		talonLF = new WPI_TalonSRX(PORTS.TALON_LF);
		talonLB = new WPI_TalonSRX(PORTS.TALON_LB);

		leftSpeedCG = new SpeedControllerGroup(talonLF, talonLB);
		rightSpeedCG = new SpeedControllerGroup(talonRF, talonRB);

		difDrive = new DifferentialDrive(leftSpeedCG, rightSpeedCG);
	}

	
	public void controllerMove(Joystick controller) {
		double leftAxis = controller.getRawAxis(BUTTONS.LEFT_JOY_Y_AXIS);
		double rightAxis = controller.getRawAxis(BUTTONS.RIGHT_JOY_Y_AXIS);

		double speedMultiplier = 0.69;

		//System.out.println("Left: " + leftAxis + " | Right: " + rightAxis);
		
		difDrive.tankDrive(leftAxis*speedMultiplier, rightAxis*speedMultiplier);
	}
}