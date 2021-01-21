// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;
import java.util.List;

public class Robot extends TimedRobot {
  private final Drivetrain m_drive = new Drivetrain();
  // THIS IS A REALL JANKY SOLUTION, I'D RATHER MERGE DRIVING AND DRIVETRAIN
  // but maybe abstracting auton and teleop driving wouldn't be so bad
  private final Driving drive = new Driving();

  // An example trajectory to follow during the autonomous period.
  private Trajectory m_trajectory;

  // The Ramsete Controller to follow the trajectory.
  private RamseteController m_ramseteController;

  // The timer to use during the autonomous period.
  private Timer m_timer;

  @Override
  public void robotInit() {
    // MUST BE CHANGED, I DON'T KNOW WHAT TRAJECTORY THIS IS
    // Create the trajectory to follow in autonomous. It is best to initialize
    // trajectories here to avoid wasting time in autonomous.
    m_trajectory =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            new Pose2d(3, 0, Rotation2d.fromDegrees(0)),
            new TrajectoryConfig(Units.feetToMeters(3.0), Units.feetToMeters(3.0)));
  }

  @Override
  public void autonomousInit() {
    // Initialize the timer.
    m_timer = new Timer();
    m_timer.start();

    // Reset the drivetrain's odometry to the starting pose of the trajectory.
    m_drive.resetOdometry(m_trajectory.getInitialPose());
  }

  @Override
  public void autonomousPeriodic() {
    // Update odometry.
    m_drive.updateOdometry();

    if (m_timer.get() < m_trajectory.getTotalTimeSeconds()) {
      // Get the desired pose from the trajectory.
      var desiredPose = m_trajectory.sample(m_timer.get());

      // Get the reference chassis speeds from the Ramsete controller.
      var refChassisSpeeds = m_ramseteController.calculate(m_drive.getPose(), desiredPose);

      // Set the linear and angular speeds.
      m_drive.drive(refChassisSpeeds.vxMetersPerSecond, refChassisSpeeds.omegaRadiansPerSecond);
    } else {
      m_drive.drive(0, 0);
    }
  }

  @Override
  public void teleopPeriodic() {
    drive.periodic();
  }
}
