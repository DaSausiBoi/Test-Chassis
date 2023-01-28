// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public final class Constants {
    /** USB ports on the computer */
    public static final class UsbConstants {

        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int AUXDRIVER_CONTROLLER_PORT = 1;
        public static final int USB_PORT_2 = 2;
        public static final int USB_PORT_3 = 3;
    }

    /** Xbox buttons to their corresponding integer value. */
    public static final class XboxConstants {

        // Button mappings
        public static int D_PAD = 0;
        public static int A_BUTTON = 1;
        public static int B_BUTTON = 2;
        public static int X_BUTTON = 3;
        public static int Y_BUTTON = 4;
        public static int LB_BUTTON = 5;
        public static int RB_BUTTON = 6;
        public static int BACK_BUTTON = 7;
        public static int START_BUTTON = 8;
        public static int LEFT_STICK = 9;
        public static int RIGHT_STICK = 10;

        // Axis control mappings
        // Notes:
        // - Left and right trigger use axis 3
        // - Left trigger range: 0 to 1
        // - Right trigger range: 0 to -1).
        public static int LEFT_AXIS_X = 6;
        public static int LEFT_AXIS_Y = 1;
        public static int LEFT_TRIGGER_AXIS = 2;
        public static int RIGHT_TRIGGER_AXIS = 3;
        public static int RIGHT_AXIS_X = 4;
        public static int RIGHT_AXIS_Y = 5;

        // Direction pad lookup angles
        public static int POV_UP = 0;
        public static int POV_RIGHT = 90;
        public static int POV_DOWN = 180;
        public static int POV_LEFT = 270;
    }

    public static final class AutoConstants {
        // TODO: Change these values
        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or
        // theoretically
        // for *your* robot's drive.
        // The Robot Characterization Toolsuite provides a convenient tool for obtaining
        // these
        // values for your robot.
        public static final double ksVolts = 0.22;
        public static final double kvVoltSecondsPerMeter = 1.98;
        public static final double kaVoltSecondsSquaredPerMeter = 0.2;

        // Example value only - as above, this must be tuned for your drive!
        public static final double kPDriveVel = 8.5;

        public static final double kTrackwidthMeters = 0.69;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
                kTrackwidthMeters);

        public static final double kMaxSpeedMetersPerSecond = .5;
        public static final double kMaxAccelerationMetersPerSecondSquared = .25;

        // Reasonable baseline values for a RAMSETE follower in units of meters and
        // seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }

    /** Values relating to the robot's drive train */
    public static final class DriveConstants {

        // middle to middle of the wheel
        public static final double TRACK_WIDTH_METERS = 0.638;
        public static final int MAG_ENCODER_CPR = 4096;
        // public static final int TALONFX_ENCODER_CPR = 2048;
        // !remove from calculations if not using an integrated encoder
        // public static final double GEAR_RATIO = 11.25; // 12:50 => 20:54 on a
        // falconfx gives 14.8 fps. Driven/Driver
        public static final double WHEEL_DIAMETER_METERS = 0.09525;
        public static final double WHEEL_CIRCUMFERENCE_METERS = Math.PI * WHEEL_DIAMETER_METERS;
        public static final int TIMEOUT = 50;

        public static final boolean RIGHT_INVERTED = true;
        public static final boolean LEFT_INVERTED = false;

    }

    /**
     * Mapping between physical CAN devices and their corresponding ID in the CAN
     * chain
     */
    public static final class CanIdConstants {

        // Note: RIO is always 0
        // Note: PDH is always 1
        public static final int RIGHT_LEADER_ID = 2;
        public static final int RIGHT_FOLLOWER_ID = 3;
        public static final int LEFT_LEADER_ID = 4;
        public static final int LEFT_FOLLOWER_ID = 5;
    }
}