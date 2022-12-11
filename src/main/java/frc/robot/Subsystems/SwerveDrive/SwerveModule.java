package frc.robot.Subsystems.SwerveDrive;

import java.util.LinkedHashMap;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.team4646.PID;
import frc.team4646.ShuffleboardHelpers;

public class SwerveModule
{
    private final SwerveMotorDrive driveMotor;
    private final SwerveMotorTurn turnMotor;
    
    /**
     * Constructs a SwerveModule with a drive motor, turning motor, and turning encoder.
     *
     * @param driveMotorChannel CAN ID for the drive motor
     * @param turningMotorChannel CAN ID for the turning motor
     * @param turningEncoderChannel CAN ID for the turning encoder
     */
    public SwerveModule(String name, int driveMotorChannel, int turningMotorChannel, int turningEncoderChannel, double offset)
    {
        ModuleConfiguration moduleConfig = Mk4i.MK4I_L2;

        PID drivePID = new PID(.06, 0, 0, 0.04535);
        driveMotor = new SwerveMotorDrive(name, driveMotorChannel, drivePID, moduleConfig);

        PID turningPID = new PID(0.2, 0, 0.1, 0);
        turnMotor = new SwerveMotorTurn(name, turningMotorChannel, turningEncoderChannel, turningPID, moduleConfig, offset);
        
        var valueMap = new LinkedHashMap<String, DoubleSupplier>();
        valueMap.put("Current Angle", () -> Math.toDegrees(turnMotor.getMotorCurrentAngle()));
        valueMap.put("Target Angle", () -> Math.toDegrees(turnMotor.getReferenceAngle()));
        valueMap.put("Absolute Encoder Angle", () -> Math.toDegrees(turnMotor.getEncoderAbsoluteAngle()));
        valueMap.put("Offset Angle", () -> offset);
        valueMap.put("Current Velocity", () -> driveMotor.getActualVelocity());
        valueMap.put("Target Velocity", () -> driveMotor.getDesiredVelocity());
        ShuffleboardHelpers.Create_TabWithGrids("Swerve Modules", name, valueMap);
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState)
    {
        boolean invertDrive = turnMotor.setDesiredAngle(desiredState.angle);
        driveMotor.setDesiredVelocity(desiredState.speedMetersPerSecond, invertDrive);
    }

    /**
     * Get the current state of the module
     * @return
     */
    public SwerveModuleState getState()
    {
        return new SwerveModuleState(driveMotor.getActualVelocity(), new Rotation2d(turnMotor.getEncoderAbsoluteAngle()));
    }

    public void whileDisabled() {
        // driveMotor.updatePID();
        // turnMotor.updatePID();
    }
}
