// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.kauailabs.navx.frc.AHRS;

/** Represents a differential drive style drivetrain. */
public class Drivetrain {
  public static final double kMaxSpeed = 1.0; // meters per second
  public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second

  private static final double kTrackWidth = 0.7757; // meters
  private static final double kWheelRadius = 0.2032 / 2; // meters
  private static final int kEncoderResolution = 2048;

  private final MotorType kBrushless = MotorType.kBrushless;

  private final CANSparkMax sparkLF = new CANSparkMax(PORTS.SPARK_LF, kBrushless);
  private final CANSparkMax sparkLB = new CANSparkMax(PORTS.SPARK_LB, kBrushless);
  private final CANSparkMax sparkRF = new CANSparkMax(PORTS.SPARK_RF, kBrushless);
  private final CANSparkMax sparkRB = new CANSparkMax(PORTS.SPARK_RB, kBrushless);

  private final CANEncoder m_leftEncoder = sparkLF.getEncoder();;
  private final CANEncoder m_rightEncoder = sparkRF.getEncoder();;

  private final SpeedControllerGroup m_leftGroup =
      new SpeedControllerGroup(sparkLF, sparkLB);
  private final SpeedControllerGroup m_rightGroup =
      new SpeedControllerGroup(sparkRF, sparkRB);


  // MUST BE CHANGED TO NAVX OR REALSENSE
  // private final AnalogGyro m_gyro = new AnalogGyro(0);
  private final AHRS m_gyro = new AHRS(SerialPort.Port.kUSB1);

  private final PIDController m_leftPIDController = new PIDController(1, 0, 0);
  private final PIDController m_rightPIDController = new PIDController(1, 0, 0);

  private final DifferentialDriveKinematics m_kinematics =
      new DifferentialDriveKinematics(kTrackWidth);

  private final DifferentialDriveOdometry m_odometry;

  // MUST BE CHANGED I DON'T KNOW HOW THOUGH
  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);

  /**
   * Constructs a differential drive object. Sets the encoder distance per pulse and resets the
   * gyro.
   */
  public Drivetrain() {
    m_gyro.reset();

    // Set the distance per pulse for the drive encoders. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    m_leftEncoder.setPositionConversionFactor(2 * Math.PI * kWheelRadius / kEncoderResolution);
    m_rightEncoder.setPositionConversionFactor(2 * Math.PI * kWheelRadius / kEncoderResolution);

    //Reset Encoders? idk if this will work
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);

    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
  }

  /**
   * Sets the desired wheel speeds.
   *
   * @param speeds The desired wheel speeds.
   */
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    final double leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);

    final double leftOutput =
        m_leftPIDController.calculate(m_leftEncoder.getVelocity(), speeds.leftMetersPerSecond);
    final double rightOutput =
        m_rightPIDController.calculate(m_rightEncoder.getVelocity(), speeds.rightMetersPerSecond);
    m_leftGroup.setVoltage(leftOutput + leftFeedforward);
    m_rightGroup.setVoltage(rightOutput + rightFeedforward);
  }

  /**
   * Drives the robot with the given linear velocity and angular velocity.
   *
   * @param xSpeed Linear velocity in m/s.
   * @param rot Angular velocity in rad/s.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double rot) {
    var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
    setSpeeds(wheelSpeeds);
  }

  /** Updates the field-relative position. */
  public void updateOdometry() {
    m_odometry.update(
        m_gyro.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());
  }

  /**
   * Resets the field-relative position to a specific location.
   *
   * @param pose The position to reset to.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  /**
   * Returns the pose of the robot.
   *
   * @return The pose of the robot.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  
}
