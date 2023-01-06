package frc.robot.subsystems.swerve;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.team4646.LazyTalonFX;
import frc.team4646.TalonUtil;

/**
 * Wraps the Falcon motor and CANcoder for swerve drive turning
 */
public class SwerveMotorSteer
{
    private static final double MOTOR_ENCODER_OUT_OF_SYNC_LIMIT_DEGREES = 2;

    private final SwerveModuleConfig moduleConfig;
    private final TalonFX motor;
    private final CANCoder encoder;

    private boolean initialized = false;
    private double resetIteration = 0;

    private double lastDesiredAngle;

    public SwerveMotorSteer(SwerveModuleConfig config)
    {
        moduleConfig = config;

        encoder = new CANCoder(moduleConfig.cancoderID, Constants.SWERVE_DRIVE.CAN_BUS_NAME);
        encoder.configFactoryDefault();
        encoder.configAllSettings(Constants.SWERVE_DRIVE.ENCODER_CONFIG);

        motor = new LazyTalonFX(moduleConfig.turnID, Constants.SWERVE_DRIVE.CAN_BUS_NAME);
        motor.configFactoryDefault();
        motor.configAllSettings(Constants.SWERVE_DRIVE.STEER_CONFIG);
        motor.setInverted(Constants.SWERVE_DRIVE.SDS_MODULE_CONFIG.angleMotorInvert);
        motor.setNeutralMode(NeutralMode.Coast);
    }

    /**
     * Will calibrate the encoder with the stored offset angle when it's available.
     * Call from the subsystem's periodic function.
     */
    public void periodic() {
        // if enabled before it calibrated, keep trying
        if(!initialized) {
            if(calibrateOffsetAngle()) {
                // this will let first enable not rotate to 0 degrees
                lastDesiredAngle = getRawEncoderAngle().minus(moduleConfig.offset).getDegrees();
            }
        }

        if (Constants.SWERVE_DRIVE.STEER_IDLE_RESYNC_ENCODER) {
            // automatic recalibration if sitting there idle
            if (TalonUtil.falconToRPM(motor.getSelectedSensorVelocity(), Constants.SWERVE_DRIVE.SDS_MODULE_CONFIG.angleGearRatio) < .1) {
                if (++resetIteration >= 500) {
                    resetIteration = 0;
                    calibrateOffsetAngle();
                }
            } else {
                resetIteration = 0;
            }
        }
    }
    
    /**
     * Set the desired angle
     * @param desiredState
     */
    public void setDesiredAngle(SwerveModuleState desiredState) {
        lastDesiredAngle = desiredState.angle.getDegrees();
        motor.set(ControlMode.Position, TalonUtil.degreesToFalcon(lastDesiredAngle, Constants.SWERVE_DRIVE.SDS_MODULE_CONFIG.angleGearRatio));
    }

    /**
     * Get the angle of the wheel as reported by the motor. Should be 0 when everything is calibrated.
     * Will increment/decrement past 360 as the wheel completes a full revolution.
     * @return -inf to +inf angle of the wheel
     */
    public Rotation2d getMotorAngle() {
        return Rotation2d.fromDegrees(TalonUtil.falconToDegrees(motor.getSelectedSensorPosition(), Constants.SWERVE_DRIVE.SDS_MODULE_CONFIG.angleGearRatio));
    }

    /**
     * Get the absoulte angle as reported by the encoder. Used to calibrate modules to 0 degrees.
     * Does not account for the offset angle
     * Wraps around at 360 degrees.
     * @return 0 to 360 angle of the wheel
     */
    public Rotation2d getRawEncoderAngle() {
        return Rotation2d.fromDegrees(encoder.getAbsolutePosition());
    }
    /**
     * Get the absoulte angle as reported by the encoder, with the offset angle applied.
     * Wraps around at 360 degrees.
     * @return 0 to 360 angle of the wheel
     */
    public Rotation2d getAbsEncoderAngle() {
        return getRawEncoderAngle().minus(moduleConfig.offset);
    }

    /**
     * Send the calibration value to the motor for aligning it to the encoder
     * @return true if position was written successfully
     */
    private boolean calibrateOffsetAngle() {
        // get the angle, 0 to 360, of what the encoder thinks our current angle is
        double angleWithOffset = getRawEncoderAngle().minus(moduleConfig.offset).getDegrees();
        // calculate what that angle is in Talon units
        double absoluteMotorPosition = TalonUtil.degreesToFalcon(angleWithOffset, Constants.SWERVE_DRIVE.SDS_MODULE_CONFIG.angleGearRatio);
        // send it to the motor to seed the initial position
        initialized = motor.setSelectedSensorPosition(absoluteMotorPosition) == ErrorCode.OK; // OK indicates the set was successful
        return initialized;
    }

    /**
     * If the initial calibration offset was sent to the motor.
     * @return
     */
    public boolean isInitialized() {
        return initialized;
    }

    /**
     * If the motor reported angle is close to the encoder reported angle
     * @return true if the motor and encoder are in sync
     */
    public boolean isMotorCloseToEncoder() {
        Rotation2d encoderWithOffset = getRawEncoderAngle().minus(moduleConfig.offset);
        Rotation2d difference = getMotorAngle().minus(encoderWithOffset); // compare the difference between the motor and the encoder's angles to see how close they're tracking
        return Math.abs(difference.getDegrees()) < MOTOR_ENCODER_OUT_OF_SYNC_LIMIT_DEGREES;
    }

    /**
     * Set a new offset angle
     * @param newAngle
     */
    public void setOffset(Rotation2d newAngle) {
        // only store if the offset has changed
        if(!newAngle.equals(moduleConfig.offset)) {
            moduleConfig.offset = newAngle;
            calibrateOffsetAngle();
        }
    }

    /**
     * Get the last angle that was set
     * @return
     */
    public double getLastDesiredAngle() {
        return lastDesiredAngle;
    }

    public TalonFX getMotor() {
        return motor;
    }
}
