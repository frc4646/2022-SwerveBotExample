package frc.robot.Subsystems.SwerveDrive;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.team4646.PID;
import frc.team4646.TalonFXFactory;
import frc.team4646.TalonUtil;

public class SwerveMotorTurn
{
    private static final double TICKS_PER_ROTATION = 2048.0;

    private static final int ENCODER_RESET_ITERATIONS = 500;
    private static final double ENCODER_RESET_MAX_ANGULAR_VELOCITY = Math.toRadians(0.5);

    private static final int CAN_TIMEOUT_MS = 250;
    private static final int STATUS_FRAME_GENERAL_PERIOD_MS = 250;
    
    private double referenceAngleRadians = 0.0;
    private double resetIteration = 0;

    private final double motorEncoderPositionCoefficient;
    private final double motorEncoderVelocityCoefficient;

    private boolean optimizeAngles = true;

    private final TalonFX motor;
    private final CANCoder encoder;

    private boolean directionInverted = false;

    public SwerveMotorTurn(int turningMotorChannel, int turningEncoderChannel, PID pid, ModuleConfiguration moduleConfig, double offset)
    {
        motor =  TalonFXFactory.createDefaultTalon(turningMotorChannel);
        encoder = new CANCoder(turningEncoderChannel);
        
        // configure the external encoder
        CANCoderConfiguration configEncoder = new CANCoderConfiguration();
        configEncoder.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        configEncoder.magnetOffsetDegrees = offset;
        configEncoder.sensorDirection = false; // TODO ? Direction.CLOCKWISE;
        TalonUtil.checkError(encoder.configAllSettings(configEncoder, CAN_TIMEOUT_MS), "Failed to configure CANCoder");
        TalonUtil.checkError(encoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10, CAN_TIMEOUT_MS), "Failed to configure CANCoder update rate");
        
        // calculate the relationship between the sensors and output angle
        motorEncoderPositionCoefficient = 2.0 * Math.PI / TICKS_PER_ROTATION * moduleConfig.getSteerReduction();
        motorEncoderVelocityCoefficient = motorEncoderPositionCoefficient * 10.0;

        TalonFXFactory.setPID(motor, pid);

        // apply electrical limits
        motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 60, .2), CAN_TIMEOUT_MS);
        motor.configVoltageCompSaturation(12.0);
        motor.enableVoltageCompensation(true);

        // configure the motor's sensor
        TalonUtil.checkError(motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, CAN_TIMEOUT_MS), "Failed to set Falcon 500 feedback sensor");
        motor.setSensorPhase(true);
        motor.setInverted(TalonFXInvertType.Clockwise);// TODO ? TalonFXInvertType.Clockwise);
        TalonUtil.checkError(motor.setSelectedSensorPosition(getEncoderAbsoluteAngle() / motorEncoderPositionCoefficient, 0, CAN_TIMEOUT_MS), "Failed to set Falcon 500 encoder position");

        // allow the motor to hold our current position
        motor.setNeutralMode(NeutralMode.Brake);

        // Reduce CAN status frame rates
        TalonUtil.checkError(
                motor.setStatusFramePeriod(
                        StatusFrameEnhanced.Status_1_General,
                        STATUS_FRAME_GENERAL_PERIOD_MS,
                        CAN_TIMEOUT_MS
                ),
                "Failed to configure Falcon status frame period"
        );
    }

    /**
     * handle angle optimization and then set the motor
     * @param angle
     * @return true if the drive motor needs to invert it's direction
     */
    public boolean setDesiredAngle(Rotation2d angle)
    {
        boolean invertDrive = false;

        double steerAngle = angle.getRadians();
        
        if(optimizeAngles)
        {
            // ensure it's with 360 degrees
            steerAngle %= (2.0 * Math.PI);
            if (steerAngle < 0.0) {
                steerAngle += 2.0 * Math.PI;
            }

            double difference = steerAngle - getMotorCurrentAngle();
            // Change the target angle so the difference is in the range [-pi, pi) instead of [0, 2pi)
            if (difference >= Math.PI) {
                steerAngle -= 2.0 * Math.PI;
            } else if (difference < -Math.PI) {
                steerAngle += 2.0 * Math.PI;
            }
            difference = steerAngle - getMotorCurrentAngle(); // Recalculate difference

            // If the difference is greater than 90 deg or less than -90 deg the drive can be inverted so the total
            // movement of the module is less than 90 deg
            if (difference > Math.PI / 2.0 || difference < -Math.PI / 2.0) {
                // Only need to add 180 deg here because the target angle will be put back into the range [0, 2pi)
                steerAngle += Math.PI;
                invertDrive = true;
            }
        }

        // Put the target angle back into the range [0, 2pi)
        steerAngle %= (2.0 * Math.PI);
        if (steerAngle < 0.0) {
            steerAngle += 2.0 * Math.PI;
        }

        setReferenceAngle(steerAngle);

        directionInverted = invertDrive;
        return directionInverted;
    }
    
    /**
     * calculate the motor's reference angle and set it
     * @param referenceAngleRadians
     */
    private void setReferenceAngle(double referenceAngleRadians)
    {
        double currentAngleRadians = motor.getSelectedSensorPosition() * motorEncoderPositionCoefficient;

        // Reset the motor's encoder periodically when the module is not rotating.
        // Sometimes (~5% of the time) when we initialize, the absolute encoder isn't fully set up, and we don't
        // end up getting a good reading. If we reset periodically this won't matter anymore.
        if (motor.getSelectedSensorVelocity() * motorEncoderVelocityCoefficient < ENCODER_RESET_MAX_ANGULAR_VELOCITY) {
            if (++resetIteration >= ENCODER_RESET_ITERATIONS) {
                resetIteration = 0;
                double absoluteAngle = getEncoderAbsoluteAngle();
                motor.setSelectedSensorPosition(absoluteAngle / motorEncoderPositionCoefficient);
                currentAngleRadians = absoluteAngle;
            }
        } else {
            resetIteration = 0;
        }

        double currentAngleRadiansMod = currentAngleRadians % (2.0 * Math.PI);
        if (currentAngleRadiansMod < 0.0) {
            currentAngleRadiansMod += 2.0 * Math.PI;
        }

        // The reference angle has the range [0, 2pi) but the Falcon's encoder can go above that
        double adjustedReferenceAngleRadians = referenceAngleRadians + currentAngleRadians - currentAngleRadiansMod;
        if (referenceAngleRadians - currentAngleRadiansMod > Math.PI) {
            adjustedReferenceAngleRadians -= 2.0 * Math.PI;
        } else if (referenceAngleRadians - currentAngleRadiansMod < -Math.PI) {
            adjustedReferenceAngleRadians += 2.0 * Math.PI;
        }

        motor.set(TalonFXControlMode.Position, adjustedReferenceAngleRadians / motorEncoderPositionCoefficient);

        this.referenceAngleRadians = referenceAngleRadians;
    }

    /**
     * Get the desired angle in radians
     * @return
     */
    public double getReferenceAngle()
    {
        return referenceAngleRadians;
    }

    /**
     * Get the absolute angle of the output in radians
     * @return
     */
    public double getEncoderAbsoluteAngle()
    {
        double angle = Math.toRadians(encoder.getAbsolutePosition());
        angle %= 2.0 * Math.PI;
        if (angle < 0.0) {
            angle += 2.0 * Math.PI;
        }

        return angle;
    }

    /**
     * Get the current angle of the module in radians
     * @return
     */
    public double getMotorCurrentAngle()
    {
        double motorAngleRadians = motor.getSelectedSensorPosition() * motorEncoderPositionCoefficient;
        motorAngleRadians %= 2.0 * Math.PI;
        if (motorAngleRadians < 0.0) {
            motorAngleRadians += 2.0 * Math.PI;
        }

        return motorAngleRadians;
    }

    public boolean getDirectionInverted()
    {
        return directionInverted;
    }
}
