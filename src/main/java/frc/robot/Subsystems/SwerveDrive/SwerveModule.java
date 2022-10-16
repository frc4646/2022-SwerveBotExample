// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.SwerveDrive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team4646.PID;
import frc.team4646.TalonFXFactory;
import frc.team4646.TalonUtil;


public class SwerveModule {
  private static final double kWheelRadius = 0.0508;

  private static final double kMpsToRPM = 6000 / 600.0 * 2048;
  private static final double kTurningGearRatio = 150/7;

  private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

  private final TalonFX m_driveMotor;
  private final TalonFX m_turningMotor;

  private final CANCoder m_turningEncoder;

  // Gains are for example purposes only - must be determined for your own robot!
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          1,
          0,
          0,
          new TrapezoidProfile.Constraints(
              kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));


  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
   *
   * @param driveMotorChannel CAN ID for the drive motor
   * @param turningMotorChannel CAN ID for the turning motor
   * @param turningEncoderChannel CAN ID for the turning encoder
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int turningEncoderChannel) {

    m_driveMotor = TalonFXFactory.createDefaultTalon(driveMotorChannel);
    m_turningMotor =  TalonFXFactory.createDefaultTalon(turningMotorChannel);

    PID drivePID = new PID(.06, 0, 0, 0.04535);
    
    TalonUtil.checkError(m_driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 100), "Motor11: Could not detect encoder: ");
    TalonFXFactory.setPID(m_driveMotor,drivePID);

    PID turningPID = new PID(.06, 0, 0, 0.04535);
  
    TalonUtil.checkError(m_turningMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 100), "Motor11: Could not detect encoder: ");
    TalonFXFactory.setPID(m_turningMotor,turningPID);

    m_turningEncoder = new CANCoder(turningEncoderChannel);

    m_turningMotor.configRemoteFeedbackFilter( 20, RemoteSensorSource.CANCoder, 0);
    m_turningMotor.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0);

    // m_turningEncoder = new Encoder(turningEncoderChannelA, turningEncoderChannelB);
  

    // Set the distance (in this case, angle) per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.
    // m_turningEncoder.setDistancePerPulse(2 * Math.PI / kEncoderResolution);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    // m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveMotor.getSensorCollection().getIntegratedSensorVelocity()*(600.0/2048),
                new Rotation2d(m_turningMotor.getSensorCollection().getIntegratedSensorPosition()/2048));

  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
   // SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningEncoder.getAbsolutePosition()));
    SmartDashboard.putNumber("encoderangle", m_turningEncoder.getAbsolutePosition());
   // m_driveMotor.set(ControlMode.Velocity, state.speedMetersPerSecond * kMpsToRPM);
    // m_turningMotor.set(ControlMode.Position, ((desiredState.angle.getDegrees()) / 360) * 4096 );
    double delta = desiredState.angle.minus(Rotation2d.fromDegrees(m_turningEncoder.getAbsolutePosition())).getDegrees() / 360 * 4096;
    m_turningMotor.set(ControlMode.Position, m_turningEncoder.getPosition()/m_turningEncoder.configGetFeedbackCoefficient() + delta );
    
  }
}
