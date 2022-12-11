package frc.robot.Subsystems.SwerveDrive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.team4646.PID;
import frc.team4646.PIDTuner;
import frc.team4646.TalonFXFactory;
import frc.team4646.TalonUtil;

public class SwerveMotorDrive
{

    private static final int CAN_TIMEOUT_MS = 250;
    
    private TalonFX motor;
    private PIDTuner pidTuner;

    private final double driveSensorVelocityCoefficient;
    private final double motorSensorTicksPerMeter;

    private double desiredVelocityMetersPerSecond;

    public SwerveMotorDrive(String name, int driveMotorChannel, PID pid, ModuleConfiguration moduleConfig)
    {
        motor = TalonFXFactory.createDefaultTalon(driveMotorChannel);

        // configure the motor's encoder
        TalonUtil.checkError(motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 100), "Motor11: Could not detect encoder: ");
        
        TalonFXFactory.setPID(motor, pid);
        pidTuner = new PIDTuner("Swerve Tuning", name + " Drive", pid, motor);

        // calculate the relationship between the sensor and output in mps
        double driveSensorPositionCoefficient = Math.PI * moduleConfig.getWheelDiameter() * moduleConfig.getDriveReduction() / TalonUtil.TICKS_PER_ROTATION;
        driveSensorVelocityCoefficient = driveSensorPositionCoefficient * 10.0;

        motorSensorTicksPerMeter = Math.PI * moduleConfig.getWheelDiameter() / (moduleConfig.getDriveReduction() * TalonUtil.TICKS_PER_ROTATION);

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

    public TalonFX getMotor() {
        return motor;
    }

    public double getDistance() {
        return motor.getSelectedSensorPosition() / motorSensorTicksPerMeter;
    }

    public void updatePID(){
        pidTuner.updateMotorPIDF();
    }
}
