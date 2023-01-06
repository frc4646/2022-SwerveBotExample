package frc.robot;

import frc.robot.subsystems.swerve.SDSModuleConfig;
import frc.robot.subsystems.swerve.SwerveModuleConfig;
import frc.team4646.PID;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
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

    public static final class SWERVE_DRIVE {
        public static final boolean DASHBOARD_ENABLED = true;

        public static final double MAX_SPEED = 2; // 4.5 meters per second
        public static final double MAX_ANGULAR_SPEED = Math.PI; // 1/2 rotation per second

        // dimensions
        public static final double WHEEL_TRACK_WIDTH_X_METERS = Units.inchesToMeters(25.6);
        public static final double WHEEL_TRACK_WIDTH_Y_METERS = Units.inchesToMeters(23.25);

        // gearbox configuration of the modules
        public static final SDSModuleConfig SDS_MODULE_CONFIG = SDSModuleConfig.SDSMK4i(SDSModuleConfig.Mk4i.L2);

        // which CAN bus are the swerve components on
        public static final String CAN_BUS_NAME = "rio"; // "rio" is built-in, CANivore name is set via Phoenix Tuner

        // CAN identifiers for each module
        public static final SwerveModuleConfig CONFIG_FRONT_LEFT  = new SwerveModuleConfig("FrontLeft",  11, 12, 13, Rotation2d.fromDegrees(176.3));
        public static final SwerveModuleConfig CONFIG_FRONT_RIGHT = new SwerveModuleConfig("FrontRight", 14, 15, 16, Rotation2d.fromDegrees(305.7));
        public static final SwerveModuleConfig CONFIG_BACK_LEFT   = new SwerveModuleConfig("BackLeft",   20, 21, 22, Rotation2d.fromDegrees(181.2));
        public static final SwerveModuleConfig CONFIG_BACK_RIGHT  = new SwerveModuleConfig("BackRight",  17, 18, 19, Rotation2d.fromDegrees(25.9));

        public static PID DRIVE_PID = new PID(.06, 0, 0, 0.04535);
        public static PID STEER_PID = new PID(0.2, 0, 0.1, 0);

        /** if true, when the track is idle it will try to resync the angle encoder */
        public static boolean STEER_IDLE_RESYNC_ENCODER = false;

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double DRIVE_KS = (0.32 / 12); //TODO: This must be tuned to specific robot
        public static final double DRIVE_KV = (1.51 / 12);
        public static final double DRIVE_KA = (0.27 / 12);

        public static final CANCoderConfiguration ENCODER_CONFIG;
        public static final TalonFXConfiguration STEER_CONFIG;
        public static final TalonFXConfiguration DRIVE_CONFIG;
        
        static {
            ENCODER_CONFIG = new CANCoderConfiguration();
            ENCODER_CONFIG.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
            ENCODER_CONFIG.sensorDirection = Constants.SWERVE_DRIVE.SDS_MODULE_CONFIG.canCoderInvert;
            ENCODER_CONFIG.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
            ENCODER_CONFIG.sensorTimeBase = SensorTimeBase.PerSecond;
            
            STEER_CONFIG = new TalonFXConfiguration();
            STEER_CONFIG.slot0.kP = STEER_PID.P;
            STEER_CONFIG.slot0.kI = STEER_PID.I;
            STEER_CONFIG.slot0.kD = STEER_PID.D;
            STEER_CONFIG.slot0.kF = STEER_PID.F;
            STEER_CONFIG.supplyCurrLimit = new SupplyCurrentLimitConfiguration(true, 25, 40, 0.1);
            
            DRIVE_CONFIG = new TalonFXConfiguration();
            DRIVE_CONFIG.slot0.kP = DRIVE_PID.P;
            DRIVE_CONFIG.slot0.kI = DRIVE_PID.I;
            DRIVE_CONFIG.slot0.kD = DRIVE_PID.D;
            DRIVE_CONFIG.slot0.kF = DRIVE_PID.F;        
            DRIVE_CONFIG.supplyCurrLimit = new SupplyCurrentLimitConfiguration(true, 35, 60, 0.1);
            DRIVE_CONFIG.openloopRamp = 0.25;
            DRIVE_CONFIG.closedloopRamp = 0.0;
            
        }
    }
}