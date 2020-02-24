package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj.Filesystem;


class Music {

	private boolean firstRun;
	private TalonFX falconTalon;
	private Orchestra orchestra;

	public Music() {
		firstRun = true;
		orchestra = new Orchestra();
		falconTalon = new TalonFX(PORTS.FALCON_TALON);
		orchestra.addInstrument(falconTalon);
		System.out.println("Added Falcon");
		String deployDir = Filesystem.getDeployDirectory().toString();
		System.out.println(deployDir);
		orchestra.loadMusic(deployDir + "/song2.chrp");
		System.out.println("Added Music");

	}
	public void playMusic(){
		if(firstRun)
			musicInit();
	}
	private void musicInit() {
		orchestra.play();
		System.out.println("Played Music");
		firstRun = false;
	}
}