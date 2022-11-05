package frc.robot.Subsystems.SwerveDrive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.team4646.PID;
import frc.team4646.TalonFXFactory;
import frc.team4646.TalonUtil;

public class SwerveMotorDrive
{
    private static final double TICKS_PER_ROTATION = 2048.0;

    private static final int CAN_TIMEOUT_MS = 250;
    
    private TalonFX motor;
    private final double driveSensorVelocityCoefficient;

    private double desiredVelocityMetersPerSecond;

    public SwerveMotorDrive(int driveMotorChannel, PID pid, ModuleConfiguration moduleConfig)
    {
        motor = TalonFXFactory.createDefaultTalon(driveMotorChannel);

        // configure the motor's encoder
        TalonUtil.checkError(motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 100), "Motor11: Could not detect encoder: ");
        
        TalonFXFactory.setPID(motor, pid);

        // calculate the relationship between the sensor and output in mps
        double driveSensorPositionCoefficient = Math.PI * moduleConfig.getWheelDiameter() * moduleConfig.getDriveReduction() / TICKS_PER_ROTATION;
        driveSensorVelocityCoefficient = driveSensorPositionCoefficient * 10.0;

        // apply electrical limits
        motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 80, .2), CAN_TIMEOUT_MS);
        motor.configVoltageCompSaturation(12.0);
        motor.enableVoltageCompensation(true);
    }

    /**
     * Set the desired velocity, in mps
     * @param speedMetersPerSecond
     * @param invertDrive
     */
    public void setDesiredVelocity(double speedMetersPerSecond, boolean invertDrive)
    {
        if(invertDrive)
        {
            desiredVelocityMetersPerSecond = speedMetersPerSecond * -1.0;
        }
        else
        {
            desiredVelocityMetersPerSecond = speedMetersPerSecond;
        }
        
        motor.set(ControlMode.Velocity, desiredVelocityMetersPerSecond / driveSensorVelocityCoefficient );
    }
    
    /**
     * Get the current velocity for use in WPILIB state logic
     * @return
     */
    public double getState()
    {
        return getActualVelocity();
    }

    /**
     * Get the current velocity of the output, in mps
     * @return
     */
    public double getActualVelocity()
    {
        return motor.getSelectedSensorVelocity() * driveSensorVelocityCoefficient;
    }

    /**
     * Get the desired velocity, in mps
     * @return
     */
    public double getDesiredVelocity() {
        return desiredVelocityMetersPerSecond;
    }
}
