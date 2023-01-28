package frc.robot;

import static frc.robot.Constants.*;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.commands.AutoDriveBackCommand;
import frc.robot.commands.TestDriveCommand;
import frc.robot.Constants.UsbConstants;

import frc.robot.subsystems.Drivetrain;

public class RobotContainer {
  private final Drivetrain drivetrain = new Drivetrain();

  private final XboxController driverController = new XboxController(UsbConstants.DRIVER_CONTROLLER_PORT);
  private final XboxController driverController2 = new XboxController(UsbConstants.AUXDRIVER_CONTROLLER_PORT);

  private final SendableChooser<Command> m_autoChooser = new SendableChooser<Command>();

  public RobotContainer() {
    // add negative (-) to getLeftY to invert drive (shooter will be the back,
    // intake will be the front)
    configureButtonBindings();

    initializeAutoChooser();

    drivetrain.setDefaultCommand(new RunCommand(() -> drivetrain.arcadeDrive(
        driverController.getRightX(),
        driverController.getLeftY()),
        drivetrain));
  }

  private void configureButtonBindings() {
    Trigger lb = new JoystickButton(driverController, XboxConstants.LB_BUTTON);
    Trigger rb = new JoystickButton(driverController, XboxConstants.RB_BUTTON);
    Trigger a = new JoystickButton(driverController2, XboxConstants.A_BUTTON);
    Trigger b = new JoystickButton(driverController2, XboxConstants.B_BUTTON);
    Trigger x = new JoystickButton(driverController2, XboxConstants.X_BUTTON);
    Trigger y = new JoystickButton(driverController2, XboxConstants.Y_BUTTON);
    POVButton povUp = new POVButton(driverController2, 0);
    POVButton povUpRight = new POVButton(driverController2, 45);
    POVButton povRight = new POVButton(driverController2, 90);
    POVButton povDownRight = new POVButton(driverController2, 135);
    POVButton povDown = new POVButton(driverController2, 180);
    POVButton povDownLeft = new POVButton(driverController2, 225);
    POVButton povLeft = new POVButton(driverController2, 270);
    POVButton povUpLeft = new POVButton(driverController2, 315);
  }

  public void initializeAutoChooser() {
    m_autoChooser.setDefaultOption("Test", new TestDriveCommand(drivetrain));
    m_autoChooser.addOption("Drive Back", new WaitCommand(0.1)
        .andThen(new AutoDriveBackCommand(drivetrain).withTimeout(3.8)));

    SmartDashboard.putData("Auto Selector", m_autoChooser);
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return m_autoChooser.getSelected();


    // Reset at the beginning of auto (for testing)
    drivetrain.resetEncoders();
    drivetrain.zeroHeading();


    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(
            AutoConstants.ksVolts,
            AutoConstants.kvVoltSecondsPerMeter,
            AutoConstants.kaVoltSecondsSquaredPerMeter),
        AutoConstants.kDriveKinematics,
        10);

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(AutoConstants.kDriveKinematics)
        // Apply the voltage constraint
        .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            // new Translation2d(1, 0),
            new Translation2d(0.5, 0)
          ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(1, 0, new Rotation2d(0)),
        // Pass config
        config);

    RamseteCommand ramseteCommand = new RamseteCommand(
        exampleTrajectory,
        drivetrain::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(
            AutoConstants.ksVolts,
            AutoConstants.kvVoltSecondsPerMeter,
            AutoConstants.kaVoltSecondsSquaredPerMeter),
        AutoConstants.kDriveKinematics,
        drivetrain::getWheelSpeeds,
        new PIDController(AutoConstants.kPDriveVel, 0, 0),
        new PIDController(AutoConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        drivetrain::tankDriveVolts,
        drivetrain);


         // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory2 = TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      new Pose2d(1, 0, new Rotation2d(0)),
      // Pass through these two interior waypoints, making an 's' curve path
      List.of(
          // new Translation2d(1, 0),
          new Translation2d(0.5, 0)
        ),
      // End 3 meters straight ahead of where we started, facing forward
      new Pose2d(0, 0, new Rotation2d(0)),
      // Pass config
      config);

  RamseteCommand ramseteCommand2 = new RamseteCommand(
      exampleTrajectory2,
      drivetrain::getPose,
      new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
      new SimpleMotorFeedforward(
          AutoConstants.ksVolts,
          AutoConstants.kvVoltSecondsPerMeter,
          AutoConstants.kaVoltSecondsSquaredPerMeter),
      AutoConstants.kDriveKinematics,
      drivetrain::getWheelSpeeds,
      new PIDController(AutoConstants.kPDriveVel, 0, 0),
      new PIDController(AutoConstants.kPDriveVel, 0, 0),
      // RamseteCommand passes volts to the callback
      drivetrain::tankDriveVolts,
      drivetrain);

    // Reset odometry to the starting pose of the trajectory.
    drivetrain.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> ramseteCommand2.andThen(() -> drivetrain.tankDriveVolts(0, 0)));
  }

}
