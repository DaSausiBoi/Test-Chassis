package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

public class Drivetrain extends SubsystemBase {
    private final WPI_TalonSRX leftLeader = new WPI_TalonSRX(CanIdConstants.LEFT_LEADER_ID);
    private final WPI_TalonSRX rightLeader = new WPI_TalonSRX(CanIdConstants.RIGHT_LEADER_ID);
    private final WPI_VictorSPX leftFollower = new WPI_VictorSPX(CanIdConstants.LEFT_FOLLOWER_ID);
    private final WPI_VictorSPX rightFollower = new WPI_VictorSPX(CanIdConstants.RIGHT_FOLLOWER_ID);
  
    private final DifferentialDrive differentialDrive = new DifferentialDrive(leftLeader, rightLeader);

  public Drivetrain() {
    leftLeader.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, DriveConstants.TIMEOUT);
    rightLeader.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, DriveConstants.TIMEOUT);

    leftLeader.setNeutralMode(NeutralMode.Brake);
    rightLeader.setNeutralMode(NeutralMode.Brake);
    rightFollower.setNeutralMode(NeutralMode.Brake);
    leftFollower.setNeutralMode(NeutralMode.Brake);
        
    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);
  }

  public void arcadeDrive(double speed, double direction){
    differentialDrive.arcadeDrive(speed, direction);
  }
}