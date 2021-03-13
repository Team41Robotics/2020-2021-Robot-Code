package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.Joystick;

class Indexer {
    private final MotorType kBrushless = MotorType.kBrushless;
    private TalonFX spindexer, shooterTest;
    private TalonSRX primer;
    private CANSparkMax littleOmni, bigOmni, elevLeft, elevRight;

    private double vertSpeed = 0, spindexSpeed = 0, primerSpeed = 0;

    private double startTime;

    private boolean jammed = false;

    public static Joystick driverstation = new Joystick(2);
    
    

	public Indexer() {
        littleOmni = new CANSparkMax(PORTS.LITTLE_OMNI_SPARK, kBrushless);
        bigOmni = new CANSparkMax(PORTS.BIG_OMNI_SPARK, kBrushless);
        elevLeft = new CANSparkMax(PORTS.ELEVATOR_SPARK_LEFT, kBrushless);
        elevRight = new CANSparkMax(PORTS.ELEVATOR_SPARK_RIGHT, kBrushless);

        primer = new TalonSRX(PORTS.SHOOTER_TALON);
        spindexer = new TalonFX(PORTS.SPINDEXER_FALCON);
        shooterTest = new TalonFX(PORTS.SHOOTER_FALCON);
	}

	private void indexerInit() {
		
	}

	public void periodic() {
		SmartDashboard.putNumber("indexer falcon current", spindexer.getSupplyCurrent());
        //System.out.println(spindexer.getSupplyCurrent());
        //System.out.println("spindex speed: " + spindexSpeed);

		if(spindexer.getSupplyCurrent() > 3){
            jammed = true;
            startTime = System.currentTimeMillis();   
        }

        // System.out.println("jammed: " + jammed);
        //System.out.println("Left Toggle: " + driverstation.getRawButton(BUTTONS.DRIVER_STATION.TOGGLE_SWITCH_L));
        //System.out.println("Middle Toggle: " + driverstation.getRawButton(BUTTONS.DRIVER_STATION.TOGGLE_SWITCH_M));
        //System.out.println("Right Toggle: " + driverstation.getRawButton(BUTTONS.DRIVER_STATION.TOGGLE_SWITCH_R));

        if(jammed){
            if(System.currentTimeMillis()- startTime < 500)
                spindexSpeed = 0;
            else
                jammed = false;
        }
        else if(!jammed && driverstation.getRawButton(BUTTONS.DRIVER_STATION.TOGGLE_SWITCH_L)){
            //spindexSpeed = -0.20;
            spindexSpeed = -(driverstation.getRawAxis(BUTTONS.DRIVER_STATION.LEFT_DIAL) + 1.0) / 6.0;
        }
        else {
            spindexSpeed = 0;
        }

        if(driverstation.getRawButton(BUTTONS.DRIVER_STATION.TOGGLE_SWITCH_M)){
            vertSpeed = 0.2;
        }
        else {
            vertSpeed = 0.0;
        }

        if(driverstation.getRawButton(BUTTONS.DRIVER_STATION.TOGGLE_SWITCH_R)){
            primerSpeed = 0.8;
        }
        else {
            primerSpeed = 0.0;
        }
        System.out.println("Spindex Speed: " + spindexSpeed);
        System.out.println("Shooter Speed: " + (driverstation.getRawAxis(BUTTONS.DRIVER_STATION.LEFT_SLIDER) + 1.0) / 2.0);
        
        spindexer.set(ControlMode.PercentOutput, spindexSpeed);
        primer.set(ControlMode.PercentOutput, -primerSpeed);
        shooterTest.set(ControlMode.PercentOutput, -(driverstation.getRawAxis(BUTTONS.DRIVER_STATION.LEFT_SLIDER) + 1.0) / 2.0);

        littleOmni.set(vertSpeed*6); // 2 inch
        bigOmni.set(vertSpeed*3); // 4 inch
        elevLeft.set(-vertSpeed*2); // 3 inch
        elevRight.set(vertSpeed*2); // 3 inch
	}

	private void auton() {
		
    }

}