package frc.robot.Subsystems.SwerveDrive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.team4646.PID;

public class SwerveModule
{
    private final SwerveMotorDrive m_driveMotor;
    private final SwerveMotorTurn turnMotor;

    ShuffleboardContainer container;

    /**
     * Constructs a SwerveModule with a drive motor, turning motor, and turning encoder.
     *
     * @param driveMotorChannel CAN ID for the drive motor
     * @param turningMotorChannel CAN ID for the turning motor
     * @param turningEncoderChannel CAN ID for the turning encoder
     */
    public SwerveModule(int driveMotorChannel, int turningMotorChannel, int turningEncoderChannel)
    {
        ModuleConfiguration moduleConfig = Mk4i.MK4I_L2;

        PID drivePID = new PID(.06, 0, 0, 0.04535);
        m_driveMotor = new SwerveMotorDrive(driveMotorChannel, drivePID, moduleConfig);

        PID turningPID = new PID(0.2, 0, 0.1, 0);
        turnMotor = new SwerveMotorTurn(turningMotorChannel, turningEncoderChannel, turningPID, moduleConfig);
    }

    /**
     * Constructs a SwerveModule with a drive motor, turning motor, turning encoder, and a Shuffleboard layout.
     *
     * @param driveMotorChannel CAN ID for the drive motor
     * @param turningMotorChannel CAN ID for the turning motor
     * @param turningEncoderChannel CAN ID for the turning encoder
     * @param layout Shuffleboard layout
     */
    public SwerveModule(int driveMotorChannel, int turningMotorChannel, int turningEncoderChannel, ShuffleboardLayout layout)
    {
        this(driveMotorChannel,turningMotorChannel,turningEncoderChannel);

        container = layout;
        container.addNumber("Current Angle", () -> Math.toDegrees(turnMotor.getMotorCurrentAngle()));
        container.addNumber("Target Angle", () -> Math.toDegrees(turnMotor.getReferenceAngle()));
        container.addNumber("Absolute Encoder Angle", () -> Math.toDegrees(turnMotor.getEncoderAbsoluteAngle()));
        container.addNumber("Current Velocity", () -> m_driveMotor.getActualVelocity());
        container.addNumber("Target Velocity", () -> m_driveMotor.getDesiredVelocity());
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState)
    {
        boolean invertDrive = turnMotor.setDesiredAngle(desiredState.angle);
        m_driveMotor.setDesiredVelocity(desiredState.speedMetersPerSecond, invertDrive);
    }

    /**
     * Get the current state of the module
     * @return
     */
    public SwerveModuleState getState()
    {
        return new SwerveModuleState(m_driveMotor.getActualVelocity(), new Rotation2d(turnMotor.getEncoderAbsoluteAngle()));
    }
}
