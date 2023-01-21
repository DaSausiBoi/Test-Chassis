package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
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

    double leftDistance = leftLeader.getSelectedSensorPosition() / 4096 * DriveConstants.WHEEL_CIRCUMFERENCE_METERS;
    double rightDistance = rightLeader.getSelectedSensorPosition() / 4096 * DriveConstants.WHEEL_CIRCUMFERENCE_METERS;
    Rotation2d gyroHeading = gyro.getRotation2d();
    System.out.println("leftDistance: " + String.valueOf(leftDistance) + "\trightDistance: " + String.valueOf(rightDistance) + "\tgyro: " + gyroHeading);
    
   m_odometry = new DifferentialDriveOdometry(
      gyroHeading,
      leftDistance, rightDistance,
        new Pose2d(5.0, 13.5, new Rotation2d()));


    differentialDrive.feed();
  }

  public void arcadeDrive(double speed, double direction) {
    differentialDrive.arcadeDrive(speed, direction);
  }
  public void test(){
    differentialDrive.feed();
    
        // m_odometry.getPoseMeters().

      Pose2d m_pose = m_odometry.update(
        gyro.getRotation2d(),
        leftLeader.getSelectedSensorPosition() / 4096 * DriveConstants.WHEEL_CIRCUMFERENCE_METERS,
        rightLeader.getSelectedSensorPosition() / 4096 * DriveConstants.WHEEL_CIRCUMFERENCE_METERS
      );
      System.out.println("x: " + String.valueOf(m_pose.getX()) + "\ty: " + String.valueOf(m_pose.getY()) + "\trotation " + m_pose.getRotation());
  }
}