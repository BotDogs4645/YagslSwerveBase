package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

public class Constants {
    public static class DrivetrainConstants {
        /* Specifies the robot's drive mode. 
         * When true, the robot will be in angular velocity.
         * Angular velocity mode uses the right stick to set the speed at which the robot is turning.
         * When false, the robot will be in direct angle mode.
         * Direct angle mode uses the right stick value as a heading, and points the robot in that direction.
        */
        public static final boolean useAngularVelocityDriveMode = true;

        /* Specifies the Driver Station port index for the driver's controller. */
        public static final int driveControllerPort = 0;
        /* Joystick deadband that will be applied to the drivetrain inputs. */
        public static final double stickDeadband = 0.1;

        /* Maximum drivetrain speed in meters per second. */
        public static final double maxSpeed = 4.5;
        /* Number of seconds to wait after the robot is disabled before setting drive motors to neutral. */
        /* Allows the robot to stop instantly, but is easier to push after the delay. */
        public static final int disableBreakTime = 10;

        /* PID Constants passed to PathPlanner. */
        public static final PIDConstants autonTranslationPID = new PIDConstants(1, 0, 0);
        public static final PIDConstants autonAnglePID = new PIDConstants(1, 0, 0);
    }
}
