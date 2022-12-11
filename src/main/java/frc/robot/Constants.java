package frc.robot;

import frc.team4646.PID;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class Constants {
    public static int CAN_TIMEOUT = 100;

    public static final String SHOW_DETAILS = "Show Details";
    static {
        SmartDashboard.putBoolean(SHOW_DETAILS, false);
    }

    public static final class CAN {
        public static final int PNEUMATIC_CONTROL_MODULE = 0, POWER_DISTRIBUTION_PANEL = 2,
                DRIVETRAIN_FL_DRIVE = 11, DRIVETRAIN_FL_TURN = 12, DRIVETRAIN_FL_ENCODER = 13;
    }

    public static final class DRIVETRAIN {
        public static final double THROTTLE_DEADBAND = 0.08;
        public static final double TURNING_DEADBAND = 0.07;

        public static final int CURRENT_LIMIT = 30;

        public static final double WHEEL_DIAMETER = 6.0;
        public static final double WHEEL_TRACK_WIDTH_INCHES = 26.0;
        public static final double GEAR_RATIO = 10.71;
        public static final double VELOCITY_MAX = 5290.0;

        public static PID PID_VOLTAGE = new PID(0.001, 0.0, 0.0); // Tuned 3/6
        public static PID PID_VELOCITY = new PID(0.0005, 0.0, 0.0, 1.0 / VELOCITY_MAX); // with crackpoint: F=0.00017,
                                                                                        // P=0.0001???
    }
}