package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

public class Drivetrain extends SubsystemBase {
  private final WPI_TalonSRX leftLeader = new WPI_TalonSRX(CanIdConstants.LEFT_LEADER_ID);
  private final WPI_TalonSRX rightLeader = new WPI_TalonSRX(CanIdConstants.RIGHT_LEADER_ID);
  private final WPI_VictorSPX leftFollower = new WPI_VictorSPX(CanIdConstants.LEFT_FOLLOWER_ID);
  private final WPI_VictorSPX rightFollower = new WPI_VictorSPX(CanIdConstants.RIGHT_FOLLOWER_ID);

  private final DifferentialDrive differentialDrive = new DifferentialDrive(leftLeader, rightLeader);
  private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();
  private DifferentialDriveOdometry m_odometry;

  public Drivetrain() {

    leftLeader.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, DriveConstants.TIMEOUT);
    rightLeader.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0,
        DriveConstants.TIMEOUT);

    leftLeader.setNeutralMode(NeutralMode.Brake);
    rightLeader.setNeutralMode(NeutralMode.Brake);
    rightFollower.setNeutralMode(NeutralMode.Brake);
    leftFollower.setNeutralMode(NeutralMode.Brake);

    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);

    rightLeader.setInverted(DriveConstants.RIGHT_INVERTED);
    rightFollower.setInverted(DriveConstants.RIGHT_INVERTED);
    leftLeader.setInverted(DriveConstants.LEFT_INVERTED);
    leftFollower.setInverted(DriveConstants.LEFT_INVERTED);

    gyro.reset();
    leftLeader.setSelectedSensorPosition(0);
    rightLeader.setSelectedSensorPosition(0);
    double leftDistance = getLeftEncoderDistance();
    double rightDistance = getRightEncoderDistance();
    Rotation2d gyroHeading = gyro.getRotation2d();
    System.out.println("leftDistance: " + String.valueOf(leftDistance) + "\trightDistance: "
        + String.valueOf(rightDistance) + "\tgyro: " + gyroHeading);

    m_odometry = new DifferentialDriveOdometry(
        gyroHeading,
        leftDistance, rightDistance,
        new Pose2d(5.0, 13.5, new Rotation2d()));

    differentialDrive.feed();
  }

  public void arcadeDrive(double speed, double direction) {
    differentialDrive.arcadeDrive(speed, direction);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        gyro.getRotation2d(), getLeftEncoderDistance(), getRightEncoderDistance());

    SmartDashboard.putNumber("heading", getHeading());
    SmartDashboard.putNumber("leftDistance", getLeftEncoderDistance());
    SmartDashboard.putNumber("leftVelocity", getLeftEncoderVelocity());
    SmartDashboard.putNumber("rightDistance", getRightEncoderDistance());
    SmartDashboard.putNumber("rightVelocity", getRightEncoderVelocity());
    SmartDashboard.putNumber("odometryHeading", m_odometry.getPoseMeters().getRotation().getDegrees());
    SmartDashboard.putNumber("odometryX", m_odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("odometryY", m_odometry.getPoseMeters().getY());

  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftLeader.setVoltage(leftVolts);
    rightLeader.setVoltage(rightVolts);
    differentialDrive.feed();
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftEncoderVelocity(), getRightEncoderVelocity());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(
        gyro.getRotation2d(),getLeftEncoderDistance(), getRightEncoderDistance(), pose);
  }

  private double getLeftEncoderDistance() {
    return leftLeader.getSelectedSensorPosition() / 4096 * DriveConstants.WHEEL_CIRCUMFERENCE_METERS;
  }

  private double getLeftEncoderVelocity() {
    return leftLeader.getSelectedSensorVelocity() / 4096 * DriveConstants.WHEEL_CIRCUMFERENCE_METERS;
  }

  private double getRightEncoderDistance() {
    return rightLeader.getSelectedSensorPosition() / 4096 * DriveConstants.WHEEL_CIRCUMFERENCE_METERS;
  }

  private double getRightEncoderVelocity() {
    return rightLeader.getSelectedSensorVelocity() / 4096 * DriveConstants.WHEEL_CIRCUMFERENCE_METERS;
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (getLeftEncoderDistance() + getRightEncoderDistance()) / 2.0;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more
   * slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    differentialDrive.setMaxOutput(maxOutput);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    leftLeader.setSelectedSensorPosition(0);
    rightLeader.setSelectedSensorPosition(0);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -gyro.getRate();
  }
}