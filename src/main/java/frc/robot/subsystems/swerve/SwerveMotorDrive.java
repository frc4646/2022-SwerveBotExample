package frc.robot.subsystems.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.team4646.LazyTalonFX;
import frc.team4646.TalonUtil;

/**
 * Wraps the Falcon motor for swerve drive propulsion
 */
public class SwerveMotorDrive
{
    private final SwerveModuleConfig moduleConfig;
    private final TalonFX motor;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.SWERVE_DRIVE.DRIVE_KS, Constants.SWERVE_DRIVE.DRIVE_KV, Constants.SWERVE_DRIVE.DRIVE_KA);
    private double desiredSpeed;

    public SwerveMotorDrive(SwerveModuleConfig config) {
        moduleConfig = config;
        motor = new LazyTalonFX(moduleConfig.driveID, Constants.SWERVE_DRIVE.CAN_BUS_NAME);
        motor.configFactoryDefault();
        motor.configAllSettings(Constants.SWERVE_DRIVE.DRIVE_CONFIG);
        motor.setInverted(Constants.SWERVE_DRIVE.SDS_MODULE_CONFIG.driveMotorInvert);
        motor.setNeutralMode(NeutralMode.Brake);
        motor.setSelectedSensorPosition(0);
    }

    /**
     * Set the desried speed
     * @param desiredState
     * @param isOpenLoop if true, use percent output mode. if false, use velocity mode
     */
    public void setDesiredSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / Constants.SWERVE_DRIVE.MAX_SPEED;
            motor.set(ControlMode.PercentOutput, percentOutput);
        }
        else {
            double velocity = TalonUtil.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.SWERVE_DRIVE.SDS_MODULE_CONFIG.wheelCircumference, Constants.SWERVE_DRIVE.SDS_MODULE_CONFIG.driveGearRatio);
            motor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
        }

        desiredSpeed = desiredState.speedMetersPerSecond;
    }

    /**
     * Get the current speed as reported by the motor in mps
     * @return 
     */
    public double getSpeed() {
        return TalonUtil.falconToMPS(motor.getSelectedSensorVelocity(), Constants.SWERVE_DRIVE.SDS_MODULE_CONFIG.wheelCircumference, Constants.SWERVE_DRIVE.SDS_MODULE_CONFIG.driveGearRatio);
    }

    /**
     * Get the desired target speed
     * @return
     */
    public double getTargetSpeed() {
        return desiredSpeed;
    }

    public TalonFX getMotor() {
        return motor;
    }
}

