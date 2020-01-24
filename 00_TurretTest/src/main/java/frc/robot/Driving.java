package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;

class Driving {
    private CANSparkMax sparkLB, sparkLF, sparkRB, sparkRF;

    public Driving() {
        sparkLB = new CANSparkMax(PORTS.SPARK_LB, MotorType.kBrushless);
        sparkLF = new CANSparkMax(PORTS.SPARK_LF, MotorType.kBrushless);
        sparkRB = new CANSparkMax(PORTS.SPARK_RB, MotorType.kBrushless);
        sparkRF = new CANSparkMax(PORTS.SPARK_RF, MotorType.kBrushless);
    }

    public void controllerMove(Joystick controller) {
        double leftJoy = controller.getRawAxis(BUTTONS.LEFT_JOY_Y_AXIS);
        double rightJoy = controller.getRawAxis(BUTTONS.RIGHT_JOY_Y_AXIS);
        
        double speedMultiplier = 0.4;
        sparkLB.set(leftJoy*speedMultiplier);
        sparkLF.set(leftJoy*speedMultiplier);
        sparkRB.set(-rightJoy*speedMultiplier);
        sparkRF.set(-rightJoy*speedMultiplier);
    }
}