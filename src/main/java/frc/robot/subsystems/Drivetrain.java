package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.motorcontrol.Victor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

/**
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.networktables.NetworkTableEntry;
import java.util.Map;
*/

public class Drivetrain extends SubsystemBase {
  private final MotorController rightLeader = new Talon(2);
  private final MotorController rightFollower = new Victor(3);
  private final MotorControllerGroup right = new MotorControllerGroup(rightLeader, rightFollower);

  private final MotorController leftLeader = new Talon(4);
  private final MotorController leftFollower = new Victor(5);
  private final MotorControllerGroup left = new MotorControllerGroup(leftLeader, leftFollower);

  private final DifferentialDrive drive = new DifferentialDrive(left, right);
  
/**
  private ShuffleboardTab tab = Shuffleboard.getTab("Main");
  public NetworkTableEntry maxSpeed = 
    tab.addPersistent("Max Speed", 1)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("min", 0, "max", 1))
      .getEntry();
  */

  public void tankDrive(double speed, double direction){
    drive.tankDrive(speed, direction);
  }
}